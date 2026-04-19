#include "test_common.h"
#include <iostream>
#include <vector>

#include <tailor/pattern.h>

void test_point_operations() {
    std::cout << "=== test_point_operations ===" << std::endl;
    
    Point2d a(0, 0);
    Point2d b(1, 1);
    Point2d c(2, 0);
    
    // 测试相对位置计算
    auto pos = tailor::CalcVectorType(1.0, 2.0);
    assert(pos == tailor::VertexRelativePositionType::RightTop);
    
    auto pos2 = tailor::CalcVectorType(-1.0, -2.0);
    assert(pos2 == tailor::VertexRelativePositionType::LeftBottom);
    
    std::cout << "test_point_operations passed!" << std::endl;
}

void test_segment_basic() {
    std::cout << "=== test_segment_basic ===" << std::endl;
    
    SimpleSegment seg1(Point2d(0, 0), Point2d(10, 10));
    SimpleSegment seg2(Point2d(0, 10), Point2d(10, 0));
    
    assert(seg1.Point0().x == 0 && seg1.Point0().y == 0);
    assert(seg1.Point1().x == 10 && seg1.Point1().y == 10);
    
    // 测试边分析器
    SimpleAnalyser analyser;
    
    // 测试起点/终点
    auto start = SimpleAnalyser::Start(seg1);
    auto end = SimpleAnalyser::End(seg1);
    assert(start.x == 0 && start.y == 0);
    assert(end.x == 10 && end.y == 10);
    
    std::cout << "test_segment_basic passed!" << std::endl;
}

void test_edge_reverse() {
    std::cout << "=== test_edge_reverse ===" << std::endl;
    
    SimpleSegment seg1(Point2d(0, 0), Point2d(10, 10));
    SimpleAnalyser analyser;
    
    auto reversed = SimpleAnalyser::Reverse(seg1);
    assert(reversed.Point0().x == 10 && reversed.Point0().y == 10);
    assert(reversed.Point1().x == 0 && reversed.Point1().y == 0);
    
    std::cout << "test_edge_reverse passed!" << std::endl;
}

void test_monotonic_split() {
    std::cout << "=== test_monotonic_split ===" << std::endl;
    
    SimpleSegment seg(Point2d(0, 0), Point2d(10, 10));
    SimpleAnalyser analyser;
    
    // 线段本身就是单调的
    auto result = analyser.SplitToMonotonic(seg);
    assert(result.monotonous.Point0().x == 0);
    assert(!result.remaining.has_value());
    
    std::cout << "test_monotonic_split passed!" << std::endl;
}

void test_tailor() {
    std::cout << "=== test_tailor (Arc) ===" << std::endl;

    // 定义弧线段类型
    using Arc = tailor::ArcSegment<Point2d, double>;
    using ArcAnalyser = tailor::ArcAnalysis<Arc, tailor::ArcSegmentAnalyserCore<Arc, tailor::PrecisionCore<6>>>;
    using TailorType = tailor::Tailor<Arc, ArcAnalyser>;

    // 创建弧线分析器
    ArcAnalyser analyser;

    std::vector<Arc> clipper;
    clipper.reserve(4);
    clipper.emplace_back(Point2d(0, 0), Point2d(4, 0), 0.1);
    clipper.emplace_back(Point2d(4, 0), Point2d(4, 4), 0.0);
    clipper.emplace_back(Point2d(4, 4), Point2d(0, 4), 0.1);
    clipper.emplace_back(Point2d(0, 4), Point2d(0, 0), 0.0);

    std::vector<Arc> subject;
    subject.reserve(4);
    subject.emplace_back(Point2d(2, 1), Point2d(6, 1), 0.0);
    subject.emplace_back(Point2d(6, 1), Point2d(6, 3), 0.08);
    subject.emplace_back(Point2d(6, 3), Point2d(2, 3), 0.0);
    subject.emplace_back(Point2d(2, 3), Point2d(2, 1), 0.05);

    // ================================
    // 创建 Tailor 并执行计算流程
    // ================================

    TailorType tailor(analyser);

    tailor.AddClipper(clipper.begin(), clipper.end());
    tailor.AddSubject(subject.begin(), subject.end());

    auto drafting = tailor.Execute();
    std::cout << "Drafting result:" << std::endl;
    std::cout << "  Edge events: " << drafting.edgeEvent.size() << std::endl;
    std::cout << "  Vertex events: " << drafting.vertexEvents.size() << std::endl;

    // 使用不同的 Pattern 进行缝合（drafting 可复用）
    using IntersectionPattern = tailor::IntersectionPattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;
    using UnionPattern = tailor::UnionPattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;
    using DifferencePattern = tailor::DifferencePattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;

    std::cout << "\nBoolean operations (same drafting, different patterns):" << std::endl;

    auto intersectionTrees = IntersectionPattern().Stitch(drafting);
    std::cout << "  Intersection: " << intersectionTrees.size() << " polygon(s)" << std::endl;

    auto unionTrees = UnionPattern().Stitch(drafting);
    std::cout << "  Union: " << unionTrees.size() << " polygon(s)" << std::endl;

    auto diffTrees = DifferencePattern().Stitch(drafting);
    std::cout << "  Difference: " << diffTrees.size() << " polygon(s)" << std::endl;

    std::cout << "test_tailor (Arc) passed!" << std::endl;
}

int main() {
    test_point_operations();
    test_segment_basic();
    test_edge_reverse();
    test_monotonic_split();
    test_tailor();
    
    std::cout << "\nAll basic tests passed!" << std::endl;
    return 0;
}
