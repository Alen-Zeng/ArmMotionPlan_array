/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MotionController.h
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   Header file 
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
#pragma once
/* Private macros ------------------------------------------------------------*/
#define MaxPointAmount 100    //轨迹点最大数量
#define JointAmount 7         //关节数量
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单个关节数据包
 * @note 包含的关节数据有：一条轨迹点中的关节位置&该位置关节的速度、电机转速&关节速度的比值
 * 
 */
typedef struct _JointDataPackStructdef
{
    float JointPosition[MaxPointAmount];          //关节位置
    float JointVelocity[MaxPointAmount];          //关节速度，与位置一一对应
}JointDataPackStructdef;


/**
 * @brief 一系列轨迹点的三次插值原式系数
 * @note 把(xk+1-x)^3称为第一项，(x-xk)^3称为第二项，(xk+1-x)称为第三项，(x-xk)称为第四项
 */
typedef struct _InterpolaCoeStructdef
{
    float FirstCoe[MaxPointAmount-1];       //第一项(xk+1-x)^3系数
    float SecCoe[MaxPointAmount-1];         //第二项(x-xk)^3系数
    float ThirdCoe[MaxPointAmount-1];       //第三项(xk+1-x)系数
    float FourthCoe[MaxPointAmount-1];      //第四项(x-xk)系数
}InterpolaCoeStructdef;


/**
 * @brief 运动控制器
 * 
 */
class MotionControllerClassdef
{
private:
    /* 三次样条插值中间变量 */
    float Hk[MaxPointAmount-1];           //使用时间间隔进行计算
    float Uk[MaxPointAmount-1];
    float LAMBDAk[MaxPointAmount-1];
    float Dk[MaxPointAmount];
    float Bk[MaxPointAmount];             //三次样条插值的情形I，对角线全为2
    /* 追赶法中间变量 */
    float pk[MaxPointAmount-1];
    float qk[MaxPointAmount];
    float yk[MaxPointAmount];
    /* 三次样条关键系数,（参见石瑞民数值计算page99） */
    float Mk[MaxPointAmount];
    /* 设置目标使用的变量 */
    bool interOK = false; //插值计算完成
    int curveNO = 0;      //第curveNO条曲线
    int tskCyclic = 1;    //设置目标函数执行的周期（单位ms）

    void chaseLUFactorization(float* bk,float* ak,float* ck,float* xk,float* dk,int size);
    template <typename T>const T &myabs(const T &input){return input < (T)0 ? -input : input;}
    void adjustDeltaX(float& _deltaX);
    bool judgeSpeedLimit(float& _tempxVariable);
    void limitSpeed(float& _tempdeltaX, float& _tempxVariable);
public:
    int pointNum;                                             //轨迹点数量
    JointDataPackStructdef jointDataPack[JointAmount];        //每一个关节的数据
    float timefromStart[MaxPointAmount];                      //到达每个轨迹点的时间点
    InterpolaCoeStructdef jointCubeInterCoe[JointAmount];     //每个时间点的各个电机的三次插值化简式系数
    float jointLinearInterCoe[JointAmount][MaxPointAmount-1]; //每个时间点的各个电机的线性插值化简式系数
    float* jointTargetptr;                                    //关节目标
    float* jointSpeedLimit[JointAmount] = {nullptr};          //关节速度上限指针（m/s 或 rad/s）
    float deltaX = 0.001;                                     //默认曲线自变量增量
    float xVariable = 0;                                      //曲线自变量（计算目标值）
    bool useCubic = false;                                    //是否使用三次插值，不使用则使用线性插值

    MotionControllerClassdef();
    MotionControllerClassdef(float* _jointTarget,int _tskCyclic);
    ~MotionControllerClassdef(){};

    /**
     * @brief 设置关节速度限制（必须）
     * 
     * @tparam Limittype 
     * @param _limits 必须是float*类型，否则出错
     */
    template <class... Limittype> void setJointSpeedLimit(Limittype*... _limits)
    {
      int limitNO = 0;
      int array[] = {(jointSpeedLimit[limitNO] = _limits, limitNO++)...};
    }
    void receiveTracjectory(int _pointNum,float* _timefromStart,float _JointsPosition[JointAmount][MaxPointAmount],float _JointsVelocity[JointAmount][MaxPointAmount] = nullptr,bool _useCubic = false);
    void interpolation();
    void jointControl();
    // void printInterCoe();
    // void GetCoe(int JointNO,int CoeNO,float* Datapool);
};


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
