#define _POSIX_C_SOURCE 200809L

#include "rknn_api.h"

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void *read_file(const char *path, uint32_t *size)
{
    FILE *file = fopen(path, "rb");
    long length;
    void *data;

    if (!file || fseek(file, 0, SEEK_END) != 0 || (length = ftell(file)) <= 0 ||
        length > UINT32_MAX || fseek(file, 0, SEEK_SET) != 0) {
        if (file) fclose(file);
        return NULL;
    }
    data = malloc((size_t)length);
    if (!data || fread(data, 1, (size_t)length, file) != (size_t)length) {
        free(data);
        fclose(file);
        return NULL;
    }
    fclose(file);
    *size = (uint32_t)length;
    return data;
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)ts.tv_nsec / 1000000U;
}

static uint64_t fnv1a(uint64_t hash, const void *data, size_t size)
{
    const uint8_t *bytes = data;
    for (size_t i = 0; i < size; ++i) hash = (hash ^ bytes[i]) * UINT64_C(1099511628211);
    return hash;
}

int main(int argc, char **argv)
{
    uint32_t model_size = 0;
    uint64_t duration_ms = 0;
    uint64_t started_ms;
    uint64_t next_report_ms;
    uint64_t expected_hash = 0;
    uint64_t runs = 0;
    void *model = NULL;
    void *input = NULL;
    rknn_output *outputs = NULL;
    rknn_context context = 0;
    rknn_input_output_num io = {0};
    rknn_tensor_attr input_attr = {0};
    rknn_sdk_version version = {0};
    int result = EXIT_FAILURE;

    if (argc < 2 || argc > 3) {
        fprintf(stderr, "usage: %s MODEL.rknn [SECONDS]\n", argv[0]);
        return EXIT_FAILURE;
    }
    if (argc == 3) {
        char *end = NULL;
        errno = 0;
        unsigned long seconds = strtoul(argv[2], &end, 10);
        if (errno || !end || *end || seconds > UINT32_MAX) {
            fprintf(stderr, "invalid duration: %s\n", argv[2]);
            return EXIT_FAILURE;
        }
        duration_ms = (uint64_t)seconds * 1000U;
    }

    model = read_file(argv[1], &model_size);
    if (!model) {
        fprintf(stderr, "cannot read model: %s\n", argv[1]);
        goto done;
    }
    if (rknn_init(&context, model, model_size, 0, NULL) != RKNN_SUCC ||
        rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)) != RKNN_SUCC ||
        io.n_input != 1 || io.n_output == 0) {
        fprintf(stderr, "RKNN model initialization failed\n");
        goto done;
    }
    input_attr.index = 0;
    if (rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr)) != RKNN_SUCC ||
        input_attr.n_elems == 0) {
        fprintf(stderr, "RKNN input query failed\n");
        goto done;
    }
    (void)rknn_query(context, RKNN_QUERY_SDK_VERSION, &version, sizeof(version));

    input = calloc(input_attr.n_elems, 1);
    outputs = calloc(io.n_output, sizeof(*outputs));
    if (!input || !outputs) goto done;

    printf("runtime=%s driver=%s model_bytes=%" PRIu32 " input_elems=%" PRIu32
           " outputs=%" PRIu32 " duration_s=%" PRIu64 "\n",
           version.api_version, version.drv_version, model_size, input_attr.n_elems,
           io.n_output, duration_ms / 1000U);

    started_ms = now_ms();
    next_report_ms = started_ms;
    do {
        rknn_input in = {0};
        rknn_perf_run perf = {0};
        uint64_t hash = UINT64_C(14695981039346656037);
        uint32_t top_index = 0;
        float top_value = 0.0f;

        in.index = 0;
        in.buf = input;
        in.size = input_attr.n_elems;
        in.type = RKNN_TENSOR_UINT8;
        in.fmt = RKNN_TENSOR_NHWC;
        for (uint32_t i = 0; i < io.n_output; ++i) {
            outputs[i].index = i;
            outputs[i].want_float = 1;
        }
        if (rknn_inputs_set(context, 1, &in) != RKNN_SUCC ||
            rknn_run(context, NULL) != RKNN_SUCC ||
            rknn_outputs_get(context, io.n_output, outputs, NULL) != RKNN_SUCC) {
            fprintf(stderr, "RKNN inference failed at run=%" PRIu64 "\n", runs + 1);
            goto done;
        }
        (void)rknn_query(context, RKNN_QUERY_PERF_RUN, &perf, sizeof(perf));
        for (uint32_t i = 0; i < io.n_output; ++i) hash = fnv1a(hash, outputs[i].buf, outputs[i].size);
        if (outputs[0].size >= sizeof(float)) {
            const float *values = outputs[0].buf;
            top_value = values[0];
            for (uint32_t i = 1; i < outputs[0].size / sizeof(float); ++i) {
                if (values[i] > top_value) {
                    top_value = values[i];
                    top_index = i;
                }
            }
        }
        if (rknn_outputs_release(context, io.n_output, outputs) != RKNN_SUCC) goto done;
        ++runs;
        if (runs == 1) expected_hash = hash;
        if (hash != expected_hash) {
            fprintf(stderr, "non-deterministic output at run=%" PRIu64 "\n", runs);
            goto done;
        }
        if (now_ms() >= next_report_ms || duration_ms == 0 || now_ms() - started_ms >= duration_ms) {
            printf("run=%" PRIu64 " elapsed_ms=%" PRIu64 " npu_us=%" PRId64
                   " output_fnv1a=%016" PRIx64 " top_index=%" PRIu32 " top_value=%.8f\n",
                   runs, now_ms() - started_ms, perf.run_duration, hash, top_index, top_value);
            fflush(stdout);
            next_report_ms = now_ms() + 60000U;
        }
    } while (duration_ms && now_ms() - started_ms < duration_ms);

    printf("S7_1_RKNN_PASS runs=%" PRIu64 " elapsed_ms=%" PRIu64 " output_fnv1a=%016" PRIx64 "\n",
           runs, now_ms() - started_ms, expected_hash);
    result = EXIT_SUCCESS;

done:
    free(outputs);
    free(input);
    if (context) rknn_destroy(context);
    free(model);
    return result;
}
