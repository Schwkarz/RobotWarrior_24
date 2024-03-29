//
// Created by RM UI Designer
//

#include "ui_default_group0_0.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 0
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group0_0;
ui_interface_round_t *ui_default_group0_null2 = (ui_interface_round_t *)&(ui_default_group0_0.data[0]);
ui_interface_round_t *ui_default_group0_null1 = (ui_interface_round_t *)&(ui_default_group0_0.data[1]);
ui_interface_round_t *ui_default_group0_shoot_flag = (ui_interface_round_t *)&(ui_default_group0_0.data[2]);
ui_interface_round_t *ui_default_group0_autoaim_flag = (ui_interface_round_t *)&(ui_default_group0_0.data[3]);
ui_interface_round_t *ui_default_group0_rotation_flag = (ui_interface_round_t *)&(ui_default_group0_0.data[4]);

void _ui_init_default_group0_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group0_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_group0_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_group0_0.data[i].figure_name[2] = i + START_ID;
        ui_default_group0_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group0_0.data[i].operate_tpyel = 0;
    }

    ui_default_group0_null2->figure_tpye = 2;
    ui_default_group0_null2->layer = 9;
    ui_default_group0_null2->r = 12;
    ui_default_group0_null2->start_x = 585;
    ui_default_group0_null2->start_y = 550;
    ui_default_group0_null2->color = 3;
    ui_default_group0_null2->width = 0;

    ui_default_group0_null1->figure_tpye = 2;
    ui_default_group0_null1->layer = 9;
    ui_default_group0_null1->r = 12;
    ui_default_group0_null1->start_x = 600;
    ui_default_group0_null1->start_y = 400;
    ui_default_group0_null1->color = 3;
    ui_default_group0_null1->width = 0;

    ui_default_group0_shoot_flag->figure_tpye = 2;
    ui_default_group0_shoot_flag->layer = 9;
    ui_default_group0_shoot_flag->r = 12;
    ui_default_group0_shoot_flag->start_x = 600;
    ui_default_group0_shoot_flag->start_y = 780;
    ui_default_group0_shoot_flag->color = 3;
    ui_default_group0_shoot_flag->width = 30;

    ui_default_group0_autoaim_flag->figure_tpye = 2;
    ui_default_group0_autoaim_flag->layer = 9;
    ui_default_group0_autoaim_flag->r = 12;
    ui_default_group0_autoaim_flag->start_x = 600;
    ui_default_group0_autoaim_flag->start_y = 680;
    ui_default_group0_autoaim_flag->color = 3;
    ui_default_group0_autoaim_flag->width = 30;

    ui_default_group0_rotation_flag->figure_tpye = 2;
    ui_default_group0_rotation_flag->layer = 9;
    ui_default_group0_rotation_flag->r = 12;
    ui_default_group0_rotation_flag->start_x = 600;
    ui_default_group0_rotation_flag->start_y = 580;
    ui_default_group0_rotation_flag->color = 3;
    ui_default_group0_rotation_flag->width = 30;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group0_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group0_0, sizeof(ui_default_group0_0));
}

void _ui_update_default_group0_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group0_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group0_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group0_0, sizeof(ui_default_group0_0));
}

void _ui_remove_default_group0_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group0_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group0_0);
    SEND_MESSAGE((uint8_t *) &ui_default_group0_0, sizeof(ui_default_group0_0));
}
