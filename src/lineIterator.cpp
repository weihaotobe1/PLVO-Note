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

#include "lineIterator.h"

//STL
#include <cmath>
#include <utility>

//Implementation of Bresenham's line drawing Algorithm
//Adapted from: https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B
// bresenham算法是计算机图形学中为了“显示器（屏幕或打印机）系由像素构成”的这个特性而设计出来的算法，
// 使得在求直线各点的过程中全部以整数来运算，因而大幅度提升计算速度。

namespace StVO {

LineIterator::LineIterator(const double x1_, const double y1_, const double x2_, const double y2_)
    : x1(x1_), y1(y1_), x2(x2_), y2(y2_), steep(std::abs(y2_ - y1_) > std::abs(x2_ - x1_)) { //steep表示y>x,直线处于陡峭的状态

    if (steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);//能够将60°变成30°
    }

    if(x1 > x2) {
        std::swap(x1, x2);//保证第二个节点在第一个节点的右侧
        std::swap(y1, y2);
    }

    dx = x2 - x1;// 表示两点之间的距离
    dy = std::abs(y2 - y1);

    error = dx / 2.0;
    ystep = (y1 < y2) ? 1 : -1;//如果第二点在右上,则为1,右下为-1

    x = static_cast<int>(x1);//由于是图像坐标,所以需要取整
    y = static_cast<int>(y1);

    maxX = static_cast<int>(x2);//右边点的坐标
}

bool LineIterator::getNext(std::pair<int, int> &pixel) {

    if (x > maxX) //说明上述操作没有正常进行
        return false;

    if (steep)//大于45°则将y和x互换
        pixel = std::make_pair(y, x);
    else
        pixel = std::make_pair(x, y);

    error -= dy;
    if (error < 0) {
        y += ystep;
        error += dx;
    }

    x++;
    return true;
}

} // namespace StVO
