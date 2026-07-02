//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef HIGH_RATE_INTERPOLATOR_H
#define HIGH_RATE_INTERPOLATOR_H

#include <rclcpp/rclcpp.hpp>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsquaternion.h>
#include <xstypes/xsvector.h>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>

struct TimedOrientation
{
    uint32_t sampleTimeFine;
    XsQuaternion quaternion;
};

struct TimedRateOfTurn
{
    uint32_t sampleTimeFine;
    XsVector gyro;
};

struct TimedAcceleration
{
    uint32_t sampleTimeFine;
    XsVector accel;
};

class HighRateInterpolator
{
public:
    HighRateInterpolator(rclcpp::Node::SharedPtr node, size_t bufferSize = 10);
    ~HighRateInterpolator();

    // Process incoming packet and return interpolated packet if ready
    bool processPacket(const XsDataPacket& packet, XsDataPacket& interpolatedPacket);

private:
    // Buffer management
    void addOrientationData(uint32_t sampleTimeFine, const XsQuaternion& quat);
    void addRateOfTurnData(uint32_t sampleTimeFine, const XsVector& gyro);
    void addAccelerationData(uint32_t sampleTimeFine, const XsVector& accel);
    
    // Interpolation methods
    bool interpolateOrientation(uint32_t targetTime, XsQuaternion& result);
    bool interpolateRateOfTurn(uint32_t targetTime, XsVector& result);
    
    // Utility functions
    int64_t computeTimeDifference(uint32_t t1, uint32_t t2);
    double computeInterpolationFactor(uint32_t targetTime, uint32_t t1, uint32_t t2);
    XsQuaternion slerp(const XsQuaternion& q1, const XsQuaternion& q2, double t);
    XsVector lerp(const XsVector& v1, const XsVector& v2, double t);
    
    // Buffer cleanup
    void cleanOldData(uint32_t currentTime);

private:
    rclcpp::Node::SharedPtr m_node;
    size_t m_bufferSize;
    
    std::mutex m_mutex;
    
    std::deque<TimedOrientation> m_orientationBuffer;
    std::deque<TimedRateOfTurn> m_rateOfTurnBuffer;
    std::deque<TimedAcceleration> m_accelerationBuffer;

    // Startup tracking
    bool m_isInitialized;
    size_t m_accelPacketCount;
    static constexpr size_t INITIALIZATION_PACKET_THRESHOLD = 50; // ~0.5s at 100Hz
    
    // Constants for timestamp wrap handling
    static constexpr uint32_t UINT32_MAX_VALUE = 0xFFFFFFFF;
    static constexpr int64_t WRAP_THRESHOLD = 0x7FFFFFFF; // Half of uint32 max
};

#endif // HIGH_RATE_INTERPOLATOR_H