/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MotionController.cpp
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   Joints' motion controller.
  * @date    2023/03/06 15:26:27
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2023-03-06 <td> 1.0 <td>zxl <td>Creator
  * </table>
  *
  ==============================================================================
                               How to use this Lib  
  ==============================================================================
    @note
      -# Joints' motion controller.
      -# 通过三次曲线的方法控制机械臂关节运动，以保证末端按照目标轨迹运动
      -# 实现的功能：
        - 对MoveIt/运动学逆解算器下发的轨迹点的关节值&目标速度进行三次插值运算（已完成）
        - 各关节速度联动，在某一关节不能达到目标速度的情况下，等比减小各关节速度，以保证末端运动轨迹不变
        - 经过单片机实测，由于单片机堆栈资源的限制和缺乏内存管理的情况，使用vector会导致内存碎片最终
          失去内存分配能力，故采用freertos的heap4内存管理，采用动态分配数组的方式进行运算
    @warning
      -# 
      -# 
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "MotionController.h"
#include <math.h>
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/


/**
 * @brief Construct a new Motion Controller Classdef:: Motion Controller Classdef object
 * 
 */
MotionControllerClassdef::MotionControllerClassdef()
{
  /* 初始化轨迹点数量为0 */
  PointNum = 0;
  /* 把对角线全部设置为2 */
  for(int i = 0;i<MaxPointAmount;i++)
  {
    Bk[i] = 2;
  }
}

/**
 * @brief Construct a new MotionControllerClassdef:: MotionControllerClassdef object
 * 
 * @param _MotorJointSpeedRatios 一系列电机关节速度比
 */
MotionControllerClassdef::MotionControllerClassdef(double* _MotorJointSpeedRatios)
{
  /* 存入电机关节速度比 */
  for(int i = 0;i<JointAmount;i++)
  {
      JointDataPack[i].MotorJointSpeedRatio = _MotorJointSpeedRatios[i];
  }
  /* 初始化轨迹点数量为0 */
  PointNum = 0;
  /* 把对角线全部设置为2 */
  for(int i = 0;i<MaxPointAmount;i++)
  {
    Bk[i] = 2;
  }
}


/**
 * @brief 接收并更新离散关节位置、各位置目标速度、各位置目标加速度和到达各轨迹点的时间点
 * @note 参数vector第一层：各个关节；第二层：某一关节的各个数据
 * @param _JointsPosition 所有关节位置
 * @param _JointsVelocity 所有关节目标速度
 * @param _JointsAcceleration 所有关节目标加速度
 * @param _TimefromStart 所有轨迹点到达时间
 */
void MotionControllerClassdef::ReceiveTracjectory(double _JointsPosition[JointAmount][MaxPointAmount],double _JointsVelocity[JointAmount][MaxPointAmount],double* _TimefromStart,int _PointNum) 
{
  /* 记录轨迹点数量 */
  PointNum = _PointNum;

  /* 第i个关节 */
  for(int i = 0;i<JointAmount;i++)
  {

    for(int j = 0;j<PointNum;j++)
    {
      /* 第j个轨迹点的关节位置 */
      JointDataPack[i].JointPosition[j] = _JointsPosition[i][j];
      /* 第j个轨迹点的关节目标速度 */
      JointDataPack[i].JointVelocity[j] = _JointsVelocity[i][j];
    }
  }
  /* 第j个轨迹点的到达时间 */
  for(int i = 0;i<PointNum;i++)
  {
    TimefromStart[i] = _TimefromStart[i];
  }
}


/**
 * @brief 三次样条插值系数计算
 * @note 每两个轨迹点之间都进行计算
 */
void MotionControllerClassdef::Interpolation() 
{
  /* 计算时间间隔Hk */
    for(int j = 0;j<PointNum-1;j++)
    {
        /* 第j个时间点，计算时间间隔，最后一个时间点不单独遍历 */
        Hk[j] = TimefromStart[j+1]-TimefromStart[j];
    }

    /* 第i个关节 */
    for(int i = 0;i<JointAmount;i++)
    {
        /* d0的计算 */
        Dk[0] = 6*((JointDataPack[i].JointPosition[1]-JointDataPack[i].JointPosition[0])/(TimefromStart[1]-TimefromStart[0]) - JointDataPack[i].JointVelocity[0])/Hk[0];
        /* 把lambdak的第一个加入为1，方便追赶法计算 */
        LAMBDAk[0] = 1;
        for(int j = 1;j<PointNum-1;j++)
        {
            /* 第j个uk，lambdak,dk,k从1到n-1 */
            Uk[j-1] = Hk[j-1]/(Hk[j-1] +Hk[j]);
            LAMBDAk[j] = 1-Uk[j-1];
            Dk[j] = 6*((JointDataPack[i].JointPosition[j+1] - JointDataPack[i].JointPosition[j])/(TimefromStart[j+1]-TimefromStart[j]) - (JointDataPack[i].JointPosition[j] - JointDataPack[i].JointPosition[j-1])/(TimefromStart[j] - TimefromStart[j-1]))/(TimefromStart[j+1] - TimefromStart[j-1]);
        }
        /* 把uk的最后一个加入为1，方便追赶法计算 */
        Uk[PointNum-2] = 1;
        /* dn的计算 */
        Dk[PointNum-1] = 6 * (JointDataPack[i].JointVelocity[PointNum - 1] - (JointDataPack[i].JointPosition[PointNum - 1] - JointDataPack[i].JointPosition[PointNum - 2]) / (TimefromStart[PointNum - 1] - TimefromStart[PointNum - 2])) / (Hk[PointNum - 2]);

        /* 求解Mk */
        ChaseLUFactorization(Bk,Uk,LAMBDAk,Mk,Dk,PointNum);

        /* 通过Mk计算系数 */
        for(int j = 0;j<PointNum-1;j++)
        {
            /* 各项系数计算 */
            JointInterCoe[i].FirstCoe[j] = Mk[j]/(6*Hk[j]);
            JointInterCoe[i].SecCoe[j] = Mk[j+1]/(6*Hk[j]);
            JointInterCoe[i].ThirdCoe[j] = (JointDataPack[i].JointPosition[j] - Mk[j]*pow(Hk[j],2)/(6))/Hk[j];
            JointInterCoe[i].FourthCoe[j] = (JointDataPack[i].JointPosition[j+1]/Hk[j] - Mk[j+1]*Hk[j]/6);
        }
    }
}


/**
 * @brief 追赶法求解线性方程组
 * 
 * @param bk 系数矩阵对角线数值
 * @param ak 系数矩阵对角线下方数值
 * @param ck 系数矩阵对角线上方数值
 * @param xk 待求解的值
 * @param dk 常数项
 */
void MotionControllerClassdef::ChaseLUFactorization(double* bk,double* ak,double* ck,double* xk,double* dk,int size)
{
    /* 开始追赶计算 */
    qk[0] = bk[0];
    for(int i = 1;i< size;i++)
    {
        pk[i-1] = ak[i-1]/qk[i-1];
        qk[i] = bk[i]-pk[i-1]*ck[i-1];
    }
    yk[0] = dk[0];
    for(int i = 1;i< size;i++)
    {
        yk[i] = dk[i]-pk[i-1]*yk[i-1];
    }
    xk[size-1] = (yk[size-1])/(qk[size-1]);
    for(int i = size-2;i>=0;i--)
    {
        /* 因为求解是倒序的，所以要从头部插入 */
        xk[i] = (yk[i]-ck[i]*xk[i+1])/qk[i];
    }
}


/**
 * @brief 打印系数
 * 
 */
void MotionControllerClassdef::PrintInterCoe()
{
  // for(int i = 0;i<JointInterCoe.Time.size();i++)
  // {
  //   std::cout<<"M"<<i<<":"<<Mk.at(i)<<"  ";
  //   std::cout<<JointInterCoe.InterpolaCoe.at(0).FirstCoe.at(i)<<"*("<<TimefromStart.at(i+1)<<"-x)^3+"<<JointInterCoe.InterpolaCoe.at(0).SecCoe.at(i)<<"*(x-"<<TimefromStart.at(i)<<")^3+"<<JointInterCoe.InterpolaCoe.at(0).ThirdCoe.at(i)<<"*("<<TimefromStart.at(i+1)<<"-x)+"<<JointInterCoe.InterpolaCoe.at(0).FourthCoe.at(i)<<"*(x-"<<TimefromStart.at(i)<<")"<<std::endl;
  // }
  // std::cout<<"Mn:"<<Mk.back()<<std::endl;
}


/**
 * @brief 获取系数结果
 * 
 * @param JointNO 哪一个关节
 * @param CoeNO 哪一项系数
 */
void MotionControllerClassdef::GetCoe(int JointNO,int CoeNO,double* Datapool)
{
  if(0<=JointNO && JointNO<JointAmount && 1<= CoeNO && CoeNO<=4)
  {
    for(int i = 0;i<PointNum-1;i++)
    {
      switch (CoeNO)
      {
      case 1:
        Datapool[i] = JointInterCoe[JointNO].FirstCoe[i];
        break;
      case 2:
        Datapool[i] = JointInterCoe[JointNO].SecCoe[i];
        break;
      case 3:
        Datapool[i] = JointInterCoe[JointNO].ThirdCoe[i];
        break;
      case 4:
        Datapool[i] = JointInterCoe[JointNO].FourthCoe[i];
        break;
      
      default:
        break;
      }
    }
  }
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
