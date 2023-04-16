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
      -# 通过线性或三次曲线的方法控制机械臂关节运动，以保证末端按照目标轨迹运动，本控制器设计主要用于适配23赛季
          工程代码的参数服务器架构，同时可以有效降低各模块的耦合度。
      -# 实现的功能：
        - 对个人设定的轨迹点进行线性插值运算
        - 对MoveIt/运动学逆解算器下发的轨迹点的关节值&目标速度进行三次插值运算
        - 各关节速度联动，在某一关节不能达到目标速度的情况下，等比减小各关节速度，以保证末端运动轨迹不变
      -# 经过单片机实测，由于单片机堆栈资源的限制和缺乏内存管理的情况，使用vector会导致内存碎片最终
         失去内存分配能力，采用freertos的heap4内存管理则由于指针难以查看内存，难以debug，故最终决定采用
         定长数组的方式进行存储和运算
      -# 使用方法：
        - 使用前，确定关节数量和最大轨迹点数量
        - 创建MotionControllerClassdef对象，并在构造函数中传入目标指针和freertos执行的频率
        - 调用setJointSpeedLimit函数设置关节速度限制
        - 调用receiveTracjectory函数接收轨迹点数据
        - 调用interpolation进行插值运算
        - 以上的代码只需要在JointControl函数调用前调用一次
        - 调用JointControl函数，该函数需要在任务中循环调用以实现增量控制
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
/* function prototypes -------------------------------------------------------*/

/**
 * @brief Construct a new Motion Controller Classdef:: Motion Controller Classdef object
 * 
 */
MotionControllerClassdef::MotionControllerClassdef()
{
  /* 初始化轨迹点数量为0 */
  pointNum = 0;
  /* 把对角线全部设置为2 */
  for(int i = 0;i<MaxPointAmount;i++)
  {
    Bk[i] = 2;
  }
}


/**
 * @brief Construct a new Motion Controller Classdef:: Motion Controller Classdef object
 * 
 * @param _jointTarget 外部关节目标数组
 */
MotionControllerClassdef::MotionControllerClassdef(float* _jointTarget,int _tskCyclic):jointTargetptr(_jointTarget),tskCyclic(_tskCyclic)
{
  /* 初始化轨迹点数量为0 */
  pointNum = 0;
  /* 把对角线全部设置为2 */
  for(int i = 0;i<MaxPointAmount;i++)
  {
    Bk[i] = 2;
  }
}


/**
 * @brief 接收并更新离散关节位置、各位置目标速度、各位置目标加速度和到达各轨迹点的时间点
 * @note 参数vector第一层：各个关节；第二层：某一关节的各个数据
 * @param _pointNum 轨迹点数量
 * @param _timefromStart 所有轨迹点到达时间
 * @param _JointsPosition 所有关节位置
 * @param _JointsVelocity 所有关节目标速度，使用线性插值时可以不传入
 * @param _useCubic 是否使用三次插值。默认为线性插值。若传入速度一定要使能这个标志位
 */
void MotionControllerClassdef::receiveTracjectory(int _pointNum,float* _timefromStart,float _JointsPosition[JointAmount][MaxPointAmount],float _JointsVelocity[JointAmount][MaxPointAmount],bool _useCubic) 
{
  if(!interOK)
  {  /* 记录轨迹点数量 */
    pointNum = _pointNum;
    useCubic = _useCubic;

    /* 第i个关节 */
    for(int i = 0;i<JointAmount;i++)
    {

      for(int j = 0;j<pointNum;j++)
      {
        /* 第j个轨迹点的关节位置 */
        jointDataPack[i].JointPosition[j] = _JointsPosition[i][j];
        /* 第j个轨迹点的关节目标速度 */
        if(useCubic)
          jointDataPack[i].JointVelocity[j] = _JointsVelocity[i][j];
      }
    }
    /* 第j个轨迹点的到达时间 */
    for(int i = 0;i<pointNum;i++)
    {
      timefromStart[i] = _timefromStart[i];
    }
  }
}


/**
 * @brief 样条插值系数计算
 * @note 每两个轨迹点之间都进行计算
 */
void MotionControllerClassdef::interpolation() 
{
  if(!interOK)
  {
    if(useCubic)
    {
      /* 计算时间间隔Hk */
      for(int j = 0;j<pointNum-1;j++)
      {
          /* 第j个时间点，计算时间间隔，最后一个时间点不单独遍历 */
          Hk[j] = timefromStart[j+1]-timefromStart[j];
      }

      /* 第i个关节 */
      for(int i = 0;i<JointAmount;i++)
      {
          /* d0的计算 */
          Dk[0] = 6*((jointDataPack[i].JointPosition[1]-jointDataPack[i].JointPosition[0])/(timefromStart[1]-timefromStart[0]) - jointDataPack[i].JointVelocity[0])/Hk[0];
          /* 把lambdak的第一个加入为1，方便追赶法计算 */
          LAMBDAk[0] = 1;
          for(int j = 1;j<pointNum-1;j++)
          {
              /* 第j个uk，lambdak,dk,k从1到n-1 */
              Uk[j-1] = Hk[j-1]/(Hk[j-1] +Hk[j]);
              LAMBDAk[j] = 1-Uk[j-1];
              Dk[j] = 6*((jointDataPack[i].JointPosition[j+1] - jointDataPack[i].JointPosition[j])/(timefromStart[j+1]-timefromStart[j]) - (jointDataPack[i].JointPosition[j] - jointDataPack[i].JointPosition[j-1])/(timefromStart[j] - timefromStart[j-1]))/(timefromStart[j+1] - timefromStart[j-1]);
          }
          /* 把uk的最后一个加入为1，方便追赶法计算 */
          Uk[pointNum-2] = 1;
          /* dn的计算 */
          Dk[pointNum-1] = 6 * (jointDataPack[i].JointVelocity[pointNum - 1] - (jointDataPack[i].JointPosition[pointNum - 1] - jointDataPack[i].JointPosition[pointNum - 2]) / (timefromStart[pointNum - 1] - timefromStart[pointNum - 2])) / (Hk[pointNum - 2]);

          /* 求解Mk */
          chaseLUFactorization(Bk,Uk,LAMBDAk,Mk,Dk,pointNum);

          /* 通过Mk计算系数 */
          for(int j = 0;j<pointNum-1;j++)
          {
              /* 各项系数计算 */
              jointCubeInterCoe[i].FirstCoe[j] = Mk[j]/(6*Hk[j]);
              jointCubeInterCoe[i].SecCoe[j] = Mk[j+1]/(6*Hk[j]);
              jointCubeInterCoe[i].ThirdCoe[j] = (jointDataPack[i].JointPosition[j] - Mk[j]*pow(Hk[j],2)/(6))/Hk[j];
              jointCubeInterCoe[i].FourthCoe[j] = (jointDataPack[i].JointPosition[j+1]/Hk[j] - Mk[j+1]*Hk[j]/6);
          }
      }
    }
    else
    {
      /* 第i个关节 */
      for(int i = 0;i<JointAmount;i++)
      {
        /* 第j个轨迹点 */
        for(int j = 0;j<pointNum-1;j++)
        {
          jointLinearInterCoe[i][j] = (jointDataPack[i].JointPosition[j+1] - jointDataPack[i].JointPosition[j])/(timefromStart[j+1] - timefromStart[j]);
        }
      }
    }
    interOK = true;
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
void MotionControllerClassdef::chaseLUFactorization(float* bk,float* ak,float* ck,float* xk,float* dk,int size)
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
 * @brief 根据最小时间间隔计算合适的deltaX
 * @note 防止一次性越过多条曲线的情况
 * @param _deltaX 默认曲线自变量增量
 */
void MotionControllerClassdef::adjustDeltaX(float& _deltaX)
{
  _deltaX = (timefromStart[1] - timefromStart[0])/3.0f;
  for(int i = 1;i<pointNum-1;i++)
  {
    (((timefromStart[i+1] - timefromStart[i])/3.0f)<_deltaX)?_deltaX=((timefromStart[i+1] - timefromStart[i])/3.0f):0;
  }
}


/**
 * @brief 判断关节速度是否超限制
 * 
 * @param _jointTargetptr 
 * @return true 有关节超速
 * @return false 没有关节超速
 */
bool MotionControllerClassdef::judgeSpeedLimit(float& _tempxVariable)
{
  float tempDeltaJointTarget = 0;
  if(_tempxVariable <= timefromStart[curveNO+1])  //目标位于当前曲线
  {
    for(int i = 0;i<JointAmount;i++)
    {   //第i个关节
      tempDeltaJointTarget = (useCubic?(jointCubeInterCoe[i].FirstCoe[curveNO]*pow((timefromStart[curveNO+1] - _tempxVariable),3) + jointCubeInterCoe[i].SecCoe[curveNO]*pow((_tempxVariable - timefromStart[curveNO]),3) + jointCubeInterCoe[i].ThirdCoe[curveNO]*(timefromStart[curveNO+1] - _tempxVariable) + jointCubeInterCoe[i].FourthCoe[curveNO]*(_tempxVariable - timefromStart[curveNO]))
                                      :(jointDataPack[i].JointPosition[curveNO]+jointLinearInterCoe[i][curveNO]*(_tempxVariable - timefromStart[curveNO])))
        - jointTargetptr[i];
      if((myabs(tempDeltaJointTarget)/tskCyclic)*1000.0f > *jointSpeedLimit[i])
      {
        return true;
      }
    }
  }
  else if(_tempxVariable > timefromStart[curveNO+1])  //目标位于下一条曲线
  {
    for(int i = 0;i<JointAmount;i++)
    {   //第i个关节
      tempDeltaJointTarget = (useCubic?(jointCubeInterCoe[i].FirstCoe[curveNO+1]*pow((timefromStart[curveNO+2] - _tempxVariable),3) + jointCubeInterCoe[i].SecCoe[curveNO+1]*pow((_tempxVariable - timefromStart[curveNO+1]),3) + jointCubeInterCoe[i].ThirdCoe[curveNO+1]*(timefromStart[curveNO+2] - _tempxVariable) + jointCubeInterCoe[i].FourthCoe[curveNO+1]*(_tempxVariable - timefromStart[curveNO+1]))
                                      :(jointDataPack[i].JointPosition[curveNO+1]+jointLinearInterCoe[i][curveNO+1]*(_tempxVariable - timefromStart[curveNO+1])))
        - jointTargetptr[i];
      if((myabs(tempDeltaJointTarget)/tskCyclic)*1000.0f > *jointSpeedLimit[i])
      {
        return true;
      }
    }
  }

  return false;   //没有关节超速
}


/**
 * @brief 关节限速
 * @note 用比例减小法（乘0.618）查找不超限的自变量 TODO:有没有更好的办法？如何判断超越的幅度？
 * @param _tempdeltaX 
 * @param _tempxVariable 
 */
void MotionControllerClassdef::limitSpeed(float& _tempdeltaX, float& _tempxVariable)
{
  while (judgeSpeedLimit(_tempxVariable))
  {
    _tempdeltaX = 0.618f*_tempdeltaX;
    _tempxVariable = xVariable + _tempdeltaX;
  }
}


/**
 * @brief 设置关节目标
 * @note 在任务中定时调用
 */
void MotionControllerClassdef::jointControl()
{
  float tempxVariable = 0;

  if(interOK && xVariable == 0)  //插值计算但未开始执行
  {
    adjustDeltaX(deltaX); //计算本次默认deltaX
  }
  float tempdeltaX = deltaX;
  if(interOK && timefromStart[0] <= xVariable && xVariable < timefromStart[pointNum -1])  //插值计算完成且正在执行
  {
    tempxVariable = xVariable + tempdeltaX;
    if(tempxVariable > timefromStart[pointNum-1])     //自变量超过末值
    {
      tempdeltaX = timefromStart[pointNum-1]-xVariable;
      tempxVariable = timefromStart[pointNum-1];
      limitSpeed(tempdeltaX,tempxVariable);
    }
    else if((limitSpeed(tempdeltaX,tempxVariable),tempxVariable > timefromStart[curveNO+1])) //跳转到下一条曲线
    {
      curveNO++;
    }
    xVariable = tempxVariable;
    /* 设置目标值 第i个关节 */
    for(int i = 0;i<JointAmount;i++)
    {
      jointTargetptr[i] = (float)(useCubic?(jointCubeInterCoe[i].FirstCoe[curveNO]*pow((timefromStart[curveNO+1] - xVariable),3) + jointCubeInterCoe[i].SecCoe[curveNO]*pow((xVariable - timefromStart[curveNO]),3) + jointCubeInterCoe[i].ThirdCoe[curveNO]*(timefromStart[curveNO+1] - xVariable) + jointCubeInterCoe[i].FourthCoe[curveNO]*(xVariable - timefromStart[curveNO]))
                                          :(jointDataPack[i].JointPosition[curveNO]+jointLinearInterCoe[i][curveNO]*(xVariable - timefromStart[curveNO])));
    }
  }
  else if(interOK && xVariable == timefromStart[pointNum-1])  //插值计算完成且执行完成
  {
    curveNO = 0;
    xVariable = 0;
    interOK = false;
  }

}



/**
 * @brief 打印系数
 * 
 */
// void MotionControllerClassdef::printInterCoe()
// {
  // for(int i = 0;i<jointCubeInterCoe.Time.size();i++)
  // {
  //   std::cout<<"M"<<i<<":"<<Mk.at(i)<<"  ";
  //   std::cout<<jointCubeInterCoe.InterpolaCoe.at(0).FirstCoe.at(i)<<"*("<<timefromStart.at(i+1)<<"-x)^3+"<<jointCubeInterCoe.InterpolaCoe.at(0).SecCoe.at(i)<<"*(x-"<<timefromStart.at(i)<<")^3+"<<jointCubeInterCoe.InterpolaCoe.at(0).ThirdCoe.at(i)<<"*("<<timefromStart.at(i+1)<<"-x)+"<<jointCubeInterCoe.InterpolaCoe.at(0).FourthCoe.at(i)<<"*(x-"<<timefromStart.at(i)<<")"<<std::endl;
  // }
  // std::cout<<"Mn:"<<Mk.back()<<std::endl;
// }


/**
 * @brief 获取系数结果
 * 
 * @param JointNO 哪一个关节
 * @param CoeNO 哪一项系数
 */
// void MotionControllerClassdef::GetCoe(int JointNO,int CoeNO,float* Datapool)
// {
//   if(0<=JointNO && JointNO<JointAmount && 1<= CoeNO && CoeNO<=4)
//   {
//     for(int i = 0;i<pointNum-1;i++)
//     {
//       switch (CoeNO)
//       {
//       case 1:
//         Datapool[i] = jointCubeInterCoe[JointNO].FirstCoe[i];
//         break;
//       case 2:
//         Datapool[i] = jointCubeInterCoe[JointNO].SecCoe[i];
//         break;
//       case 3:
//         Datapool[i] = jointCubeInterCoe[JointNO].ThirdCoe[i];
//         break;
//       case 4:
//         Datapool[i] = jointCubeInterCoe[JointNO].FourthCoe[i];
//         break;
      
//       default:
//         break;
//       }
//     }
//   }
// }

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

