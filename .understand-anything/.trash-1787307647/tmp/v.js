const r = require('E:/getfun-f722-freertos/GETFUN_F722_FreeRTOS/.understand-anything/tmp/ua-tour-results.json');
const idx = r.nodeSummaryIndex;
// print all node ids containing 'Tools' and 'ld' and 'startup' and 'RK3568/s7_6' and 's7_5'
const want = ['Tools/', 'STM32F722XX_FLASH', 'startup_stm32', 'linux_rc', 'rc_source', 'rc_task', 'platform_time', 'platform_diag'];
for (const id of Object.keys(idx)) {
  for (const w of want) {
    if (id.includes(w)) { console.log(id, '::', idx[id].name); break; }
  }
}
