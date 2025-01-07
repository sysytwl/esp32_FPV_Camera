#include <stdio.h>
#include "sensor.h"

const camera_sensor_info_t camera_sensor[CAMERA_MODEL_MAX] = {
    // The sequence must be consistent with camera_model_t
    {CAMERA_OV2640, "OV2640", OV2640_SCCB_ADDR, OV2640_PID, FRAMESIZE_UXGA, true},
    {CAMERA_OV5640, "OV5640", OV5640_SCCB_ADDR, OV5640_PID, FRAMESIZE_QSXGA, true},
    {CAMERA_NT99141, "NT99141", NT99141_SCCB_ADDR, NT99141_PID, FRAMESIZE_HD, true},
};

const resolution_info_t resolution[FRAMESIZE_INVALID] = {
    {   96,   96, ASPECT_RATIO_1X1   }, /* 96x96 */
    {  160,  120, ASPECT_RATIO_4X3   }, /* QQVGA */
    {  176,  144, ASPECT_RATIO_5X4   }, /* QCIF  */
    {  240,  176, ASPECT_RATIO_4X3   }, /* HQVGA */
    {  240,  240, ASPECT_RATIO_1X1   }, /* 240x240 */
    {  320,  240, ASPECT_RATIO_4X3   }, /* QVGA  */
    {  400,  296, ASPECT_RATIO_4X3   }, /* CIF   */
    {  480,  320, ASPECT_RATIO_3X2   }, /* HVGA  */
    {  640,  480, ASPECT_RATIO_4X3   }, /* VGA   */
    {  800,  600, ASPECT_RATIO_4X3   }, /* SVGA  */
    { 1024,  768, ASPECT_RATIO_4X3   }, /* XGA   */
    { 1280,  720, ASPECT_RATIO_16X9  }, /* HD    */
    { 1280,  960, ASPECT_RATIO_4X3   }, /* SXGA  */
    { 1600, 1200, ASPECT_RATIO_4X3   }, /* UXGA  */
    // 3MP Sensors
    { 1920, 1080, ASPECT_RATIO_16X9  }, /* FHD   */
    {  800, 456, ASPECT_RATIO_16X9  }, /* Portrait HD   changed to 800x456 */
    {  640, 360, ASPECT_RATIO_16X9  }, /* Portrait 3MP  changed to 640x360 */
    { 2048, 1536, ASPECT_RATIO_4X3   }, /* QXGA  */
    // 5MP Sensors
    { 2560, 1440, ASPECT_RATIO_16X9  }, /* QHD    */
    { 2560, 1600, ASPECT_RATIO_16X10 }, /* WQXGA  */
    { 1024, 576, ASPECT_RATIO_16X9  }, /* Portrait FHD   chaged to 1024x576 */  
    { 2560, 1920, ASPECT_RATIO_4X3   }, /* QSXGA  */
};

camera_sensor_info_t *esp_camera_sensor_get_info(sensor_id_t *id){
    for (int i = 0; i < CAMERA_MODEL_MAX; i++) {
        if (id->PID == camera_sensor[i].pid) {
            return (camera_sensor_info_t *)&camera_sensor[i];
        }
    }
    return NULL;
}
