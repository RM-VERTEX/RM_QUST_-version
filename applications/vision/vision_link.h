#ifndef APPLICATIONS__VISION__VISION_LINK_H
#define APPLICATIONS__VISION__VISION_LINK_H

/**
 * @file vision_link.h
 * @brief C-compatible declaration of the vision link task entry point.
 *
 * `Core/Src/freertos.c` and `Core/Src/main.c` are compiled as C by this
 * project (only the application layer is C++), so the task entry point is
 * declared here rather than in the C++ header.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief FreeRTOS entry point for the USB virtual COM port link task.
  * @param argument: unused
  */
void vision_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS__VISION__VISION_LINK_H */
