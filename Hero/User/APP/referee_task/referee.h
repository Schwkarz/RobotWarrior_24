#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;
typedef __packed struct // 0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct // 0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef __packed struct // 0x0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct // 0x0102
{
    // uint8_t supply_projectile_id;
    // uint8_t supply_robot_id;
    // uint8_t supply_projectile_step;
    // uint8_t supply_projectile_num;
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// V1.5已删除
// typedef __packed struct // 0x0103
// {
//     uint8_t supply_projectile_id;
//     uint8_t supply_robot_id;
//     uint8_t supply_num;
// } ext_supply_projectile_booking_t;

typedef __packed struct
{
    // uint8_t level;
    // uint8_t foul_robot_id;
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;

typedef __packed struct // 0x0201
{
    // uint8_t robot_id;
    // uint8_t robot_level;
    // uint16_t remain_HP;
    // uint16_t max_HP;
    // uint16_t shooter_id1_17mm_cooling_rate;
    // uint16_t shooter_id1_17mm_cooling_limit;
    // uint16_t shooter_id1_17mm_speed_limit;
    // uint16_t shooter_id2_17mm_cooling_rate;
    // uint16_t shooter_id2_17mm_cooling_limit;
    // uint16_t shooter_id2_17mm_speed_limit;
    // uint16_t shooter_id1_42mm_cooling_rate;
    // uint16_t shooter_id1_42mm_cooling_limit;
    // uint16_t shooter_id1_42mm_speed_limit;
    // uint16_t chassis_power_limit;
    // uint8_t mains_power_gimbal_output : 1;
    // uint8_t mains_power_chassis_output : 1;
    // uint8_t mains_power_shooter_output : 1;
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct // 0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct // 0x0203
{
    // float x;
    // float y;
    // float z;
    // float yaw;
    float x;
    float y;
    float angle;
} ext_game_robot_pos_t;

typedef __packed struct // 0x0204
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_musk_t;

typedef __packed struct // 0x0205
{
    uint8_t airforce_status;
    uint8_t time_remain;
} aerial_robot_energy_t;

typedef __packed struct // 0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct // 0x0207
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;
typedef __packed struct
{
    // uint8_t bullet_remaining_num;
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
} ext_bullet_remaining_t;
typedef __packed struct // 0x0301
{
    // uint16_t send_ID;
    // uint16_t receiver_ID;
    // uint16_t data_cmd_id;
    // uint16_t data_len;
    // uint8_t *data;
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[1];//x最大为113 
} ext_student_interactive_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef __packed struct // 0x020B
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} ext_ground_robot_position_t;

typedef __packed struct // 0x020C
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} ext_radar_mark_data_t;

typedef __packed struct // 0x0307
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} ext_map_data_t;

typedef __packed struct // 0x0306
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} ext_custom_client_data_t;

typedef __packed struct // 0x020D
{
    uint32_t sentry_info;
} ext_sentry_info_t;

typedef __packed struct // 0x020E
{
    uint8_t radar_info;
} ext_radar_info_t;

typedef __packed struct // 0x0308
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} ext_custom_info_t;

extern void
init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern void get_chassis_power_limit(fp32 *power_limit);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

extern uint8_t get_shoot_power_status(void);
extern uint8_t get_shoot_17mm_speed_limit(void);
#endif
