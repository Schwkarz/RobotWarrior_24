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

#define ui_init_default_group1() \
_ui_init_default_group1_0(); \
_ui_init_default_group1_1(); \
_ui_init_default_group1_2(); \
_ui_init_default_group1_3()

#define ui_update_default_group1() \
_ui_update_default_group1_0(); \
_ui_update_default_group1_1(); \
_ui_update_default_group1_2(); \
_ui_update_default_group1_3()

#define ui_remove_default_group1() \
_ui_remove_default_group1_0(); \
_ui_remove_default_group1_1(); \
_ui_remove_default_group1_2(); \
_ui_remove_default_group1_3()
    


#ifdef __cplusplus
}
#endif

#endif //UI_H
