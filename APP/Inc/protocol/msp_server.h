#ifndef MSP_SERVER_H
#define MSP_SERVER_H

#include "protocol/msp_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

void msp_server_init(void);
void msp_server_process(const msp_request_t *request,
                        msp_response_t *response);

#ifdef __cplusplus
}
#endif

#endif
