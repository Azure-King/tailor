#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>

#include <tailor/tailor_point.h>
#include <tailor/tailor_arc_or_segment.h>
#include <tailor/tailor.h>

// 简单的二维点类型
struct Point2d {
	double x, y;
	Point2d() = default;
	Point2d(double x, double y) : x(x), y(y) {}
};
// 点工具特化
template<>
struct tailor::PointTraits<Point2d> {
	using CoordinateType = double;
	static constexpr CoordinateType GetX(const Point2d& p) { return p.x; }
	static constexpr CoordinateType GetY(const Point2d& p) { return p.y; }
	static Point2d ConstructPoint(CoordinateType x, CoordinateType y) {
		return Point2d{ x, y };
	}
};

// 简单的矩形类型定义
using SimpleSegment = tailor::LineSegment<Point2d>;
using SimpleAnalyser = tailor::LineSegmentAnalyser<SimpleSegment, tailor::PrecisionCore<6>>;
