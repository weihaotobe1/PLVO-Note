/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#pragma once

//STL
#include <chrono>

namespace StVO {

class Timer {
public:

    // constexpr - 指定变量或函数的值可在常量表达式中出现(类似宏定义)
    static constexpr double SECONDS = 1e-9;//最高精度
    static constexpr double MILLISECONDS = 1e-6;//中等精度
    static constexpr double NANOSECONDS = 1.0;//最低精度

    Timer(double scale = MILLISECONDS);
    virtual ~Timer();

    void start();

    double stop();

private:

    std::chrono::high_resolution_clock::time_point start_t;//高精度时钟
    bool started;
    double scale;
};

} // namespace StVO
