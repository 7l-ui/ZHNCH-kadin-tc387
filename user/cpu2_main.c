/*********************************************************************************************************************
* TC387 Opensourec Library 即（TC387 开源库）是一�?基于官方 SDK 接口的�??三方开源库
* Copyright (c) 2022 SEEKFREE 逐�?��?�技
*
* �?文件�? TC387 开源库的一部分
*
* TC387 开源库 �?免费�?�?
* 您可以根�?�?由软件基金会发布�? GPL（GNU General Public License，即 GNU通用�?共�?�可证）的条�?
* �? GPL 的�??3版（�? GPL3.0）或（您选择的）任何后来的版�?，重新发布和/或修改它
*
* �?开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更�?�细节�?�参�? GPL
*
* 您应该在收到�?开源库的同时收到一�? GPL 的副�?
* 如果没有，�?�参�?<https://www.gnu.org/licenses/>
*
* 额�?�注明：
* �?开源库使用 GPL3.0 开源�?�可证协�? 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
* 许可证副�?�? libraries 文件夹下 即�?�文件夹下的 LICENSE 文件
* 欢迎各位使用并传�?�?程序 但修改内容时必须保留逐�?��?�技的版权声明（即本声明�?
*
* 文件名称          cpu2_main
* �?司名�?          成都逐�?��?�技有限�?�?
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?          ADS v1.10.2
* 适用平台          TC387QP
* 店铺链接          https://seekfree.taobao.com/
*
* �?改�?�录
* 日期              作�?                备注
* 2022-11-04       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "guimai/guimai_board.h"
#include "ui/blue_target.h"
#pragma section all "cpu2_dsram"
// 将本�?句与#pragma section all restore�?句之间的全局变量都放在CPU1的RAM�?


// 工程导入到软件之后，应�?�选中工程然后点击refresh刷新一下之后再编译
// 工程默�?��?�置为关�?优化，可以自己右击工程选择properties->C/C++ Build->Setting
// 然后在右侧的窗口�?找到C/C++ Compiler->Optimization->Optimization level处�?�置优化等级
// 一�?默�?�新建立的工程都会默认开2级优化，因�?�大家也�?以�?�置�?2级优�?

// 对于TC系列默�?�是不支持中�?嵌�?�的，希望支持中�?嵌�?�需要在�?�?内使�? enableInterrupts(); 来开�?�?�?嵌�??
// 简单点说实际上进入�?�?后TC系列的硬件自动调用了 disableInterrupts(); 来拒绝响应任何的�?�?，因此需要我�?�?己手动调�? enableInterrupts(); 来开�?�?�?的响应�?


// **************************** 代码区域 ****************************
void core2_main(void)
{
    disable_Watchdog();                     // 关闭看门�?
    interrupt_global_enable(0);             // 打开全局�?�?
    // 此�?�编写用户代�? 例�?��?��?�初始化代码�?




    // 此�?�编写用户代�? 例�?��?��?�初始化代码�?
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        guimai_board_record_core_task();
        blue_target_camera_worker_task();
        system_delay_ms(1);
    }
}



#pragma section all restore
//by ahstu zhugeliang ltl
