//
// Created by RM UI Designer
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_default_group1_0.h"
#include "ui_default_group1_1.h"
#include "ui_default_group1_2.h"
#include "ui_default_group1_3.h"
#include "ui_default_group1_4.h"

#define ui_init_default_group1() \
_ui_init_default_group1_0(); \
_ui_init_default_group1_1(); \
_ui_init_default_group1_2(); \
_ui_init_default_group1_3(); \
_ui_init_default_group1_4()

#define ui_update_default_group1() \
_ui_update_default_group1_0(); \
_ui_update_default_group1_1(); \
_ui_update_default_group1_2(); \
_ui_update_default_group1_3(); \
_ui_update_default_group1_4()

#define ui_remove_default_group1() \
_ui_remove_default_group1_0(); \
_ui_remove_default_group1_1(); \
_ui_remove_default_group1_2(); \
_ui_remove_default_group1_3(); \
_ui_remove_default_group1_4()
    

#include "ui_default_group0_0.h"

#define ui_init_default_group0() \
_ui_init_default_group0_0()

#define ui_update_default_group0() \
_ui_update_default_group0_0()

#define ui_remove_default_group0() \
_ui_remove_default_group0_0()

#define UI_DELAY_TIME 3000
#define UI_INIT_KEYBOARD KEY_PRESSED_OFFSET_R
    


#ifdef __cplusplus
}
#endif

#endif //UI_H
