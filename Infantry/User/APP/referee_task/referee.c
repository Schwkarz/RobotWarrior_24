#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_referee_warning_t referee_warning_t;
// ext_supply_projectile_booking_t supply_projectile_booking_t;


ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;


//2024.3.11 V1.6版裁判系统协议新增
ext_ground_robot_position_t ground_robot_position_t;
ext_radar_mark_data_t radar_mark_data_t;
ext_map_data_t map_data_t;
ext_custom_client_data_t custom_client_data_t;
ext_sentry_info_t sentry_info_t;
ext_radar_info_t radar_info_t;
ext_custom_info_t custom_info_t;


void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    // memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));

    //2024.3.11 V1.6版裁判系统协议新增
    memset(&ground_robot_position_t,0,sizeof(ext_ground_robot_position_t));
    memset(&radar_mark_data_t,0,sizeof(ext_radar_mark_data_t));
    memset(&map_data_t,0,sizeof(ext_map_data_t));
    memset(&custom_client_data_t,0,sizeof(ext_custom_client_data_t));
    memset(&sentry_info_t,0,sizeof(ext_sentry_info_t));
    memset(&radar_info_t,0,sizeof(ext_radar_info_t));
    memset(&custom_info_t,0,sizeof(ext_custom_info_t));

}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;

        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;

        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;

        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;

        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;

        // case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        // {
        //     memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        // }
        // break;

        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;

        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;

        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;

        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;

        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;

        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;

        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;

        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;

        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            memcpy(&ground_robot_position_t, frame + index, sizeof(ext_ground_robot_position_t));
        }
        break;
        
        case RADAR_MARK_DATA_CMD_ID:
        {
            memcpy(&radar_mark_data_t, frame + index, sizeof(ext_radar_mark_data_t));
        }
        break;

        case SENTRY_INFO_DATA_CMD_ID:
        {
            memcpy(&sentry_info_t, frame + index, sizeof(ext_sentry_info_t));
        }
        break;

        case RADAR_INFO_CMD_ID:
        {
            memcpy(&radar_info_t, frame + index, sizeof(ext_radar_info_t));
        }
        break;

        case CUSTOM_CLIENT_DATA_CMD_ID:
        {
            memcpy(&custom_client_data_t, frame + index, sizeof(ext_custom_client_data_t));
        }
        break;

        case MAP_DATA_CMD_ID:
        {
            memcpy(&map_data_t, frame + index, sizeof(ext_map_data_t));
        }
        break;

        case CUSTOM_INFO_CMD_ID:
        {
            memcpy(&custom_info_t,frame+index,sizeof(ext_custom_info_t));
        }
        break;

        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}

void get_chassis_power_limit(fp32 *power_limit)
{
	*power_limit = robot_state.chassis_power_limit;
}

uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    // *heat0_limit = robot_state.shooter_id1_17mm_cooling_limit;
    *heat0_limit = robot_state.shooter_barrel_heat_limit;
    *heat0 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    // *heat1_limit = robot_state.shooter_id2_17mm_cooling_limit;
    *heat1_limit = robot_state.shooter_barrel_heat_limit;
    *heat1 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
}

uint8_t get_shoot_power_status(void)
{
    return robot_state.power_management_shooter_output;
}

uint8_t get_shoot_17mm_speed_limit(void)
{
    // return robot_state.shooter_id1_17mm_speed_limit;

    //24赛季射速上限恒定为30
    return 30;
}