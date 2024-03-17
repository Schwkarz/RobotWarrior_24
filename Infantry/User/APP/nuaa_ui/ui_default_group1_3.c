//
// Created by RM UI Designer
//

#include "ui_default_group1_3.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 3
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_group1_3;
ui_interface_line_t *ui_default_group1_line_1 = (ui_interface_line_t *)&(ui_default_group1_3.data[0]);
ui_interface_line_t *ui_default_group1_line_2 = (ui_interface_line_t *)&(ui_default_group1_3.data[1]);
ui_interface_line_t *ui_default_group1_line_3 = (ui_interface_line_t *)&(ui_default_group1_3.data[2]);
ui_interface_line_t *ui_default_group1_line4 = (ui_interface_line_t *)&(ui_default_group1_3.data[3]);
ui_interface_line_t *ui_default_group1_line5 = (ui_interface_line_t *)&(ui_default_group1_3.data[4]);

void _ui_init_default_group1_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group1_3.data[i].figure_name[0] = FRAME_ID;
        ui_default_group1_3.data[i].figure_name[1] = GROUP_ID;
        ui_default_group1_3.data[i].figure_name[2] = i + START_ID;
        ui_default_group1_3.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_group1_3.data[i].operate_tpyel = 0;
    }

    ui_default_group1_line_1->figure_tpye = 0;
    ui_default_group1_line_1->layer = 0;
    ui_default_group1_line_1->start_x = 800;
    ui_default_group1_line_1->start_y = 540;
    ui_default_group1_line_1->end_x = 1120;
    ui_default_group1_line_1->end_y = 540;
    ui_default_group1_line_1->color = 2;
    ui_default_group1_line_1->width = 5;

    ui_default_group1_line_2->figure_tpye = 0;
    ui_default_group1_line_2->layer = 0;
    ui_default_group1_line_2->start_x = 900;
    ui_default_group1_line_2->start_y = 360;
    ui_default_group1_line_2->end_x = 1020;
    ui_default_group1_line_2->end_y = 360;
    ui_default_group1_line_2->color = 2;
    ui_default_group1_line_2->width = 5;

    ui_default_group1_line_3->figure_tpye = 0;
    ui_default_group1_line_3->layer = 0;
    ui_default_group1_line_3->start_x = 850;
    ui_default_group1_line_3->start_y = 450;
    ui_default_group1_line_3->end_x = 1070;
    ui_default_group1_line_3->end_y = 450;
    ui_default_group1_line_3->color = 2;
    ui_default_group1_line_3->width = 5;

    ui_default_group1_line4->figure_tpye = 0;
    ui_default_group1_line4->layer = 0;
    ui_default_group1_line4->start_x = 960;
    ui_default_group1_line4->start_y = 300;
    ui_default_group1_line4->end_x = 960;
    ui_default_group1_line4->end_y = 539;
    ui_default_group1_line4->color = 2;
    ui_default_group1_line4->width = 3;

    ui_default_group1_line5->figure_tpye = 0;
    ui_default_group1_line5->layer = 0;
    ui_default_group1_line5->start_x = 50;
    ui_default_group1_line5->start_y = 50;
    ui_default_group1_line5->end_x = 100;
    ui_default_group1_line5->end_y = 100;
    ui_default_group1_line5->color = 0;
    ui_default_group1_line5->width = 0;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group1_3);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_3, sizeof(ui_default_group1_3));
}

void _ui_update_default_group1_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group1_3.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group1_3);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_3, sizeof(ui_default_group1_3));
}

void _ui_remove_default_group1_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_group1_3.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_group1_3);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_3, sizeof(ui_default_group1_3));
}
