#define TAILOR_ASSERT_USE_STD
#include "test_common.h"

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <tailor/pattern.h>
#include <tailor/tailor_point.h>

#include "polygon_io.h"

// ============================================================================
//  纯标准库的多边形数据 IO（替代 Qt 版本的 PolygonIO）
//  文件格式:
//    # 注释
//    BEGIN_POLYGON N
//    startX startY endX endY bulge
//    ...
//    END_POLYGON
// ============================================================================

/** 一条边（直线或弧线） */
struct PolygonEdge {
	Point2d start;
	Point2d end;
	double bulge = 0.0;   // 0 = 线段, 非零 = 弧线
};

/** 一个多边形 = 边的序列 */
struct PolygonData {
	std::vector<PolygonEdge> edges;
};

/** 从 tokens 中解析一条边（格式: sx sy ex ey bulge） */
static bool ParseEdge(const std::vector<std::string>& tokens, PolygonEdge& out) {
	if (tokens.size() < 5) return false;
	out.start.x = std::stod(tokens[0]);
	out.start.y = std::stod(tokens[1]);
	out.end.x = std::stod(tokens[2]);
	out.end.y = std::stod(tokens[3]);
	out.bulge = std::stod(tokens[4]);
	return true;
}

/** 将一行按空白字符切分为 tokens */
static std::vector<std::string> SplitLine(const std::string& line) {
	std::vector<std::string> tokens;
	std::istringstream iss(line);
	std::string token;
	while (iss >> token)
		tokens.push_back(token);
	return tokens;
}

/** 去除首尾空白 */
static std::string Trim(const std::string& s) {
	size_t b = 0, e = s.size();
	while (b < e && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r'))
		++b;
	while (b < e && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r'))
		--e;
	return s.substr(b, e - b);
}

/**
 * 从文件中导入多边形数据
 * @param filepath  文件路径
 * @param out       输出多边形列表
 * @return 是否成功导入至少一个多边形
 */
bool ImportPolygons(const std::string& filepath, std::vector<PolygonData>& out) {
	std::ifstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "  [WARN] Cannot open file: " << filepath << std::endl;
		return false;
	}

	out.clear();
	std::vector<PolygonEdge> currentEdges;
	bool inPolygon = false;

	std::string rawLine;
	while (std::getline(file, rawLine)) {
		std::string line = Trim(rawLine);
		if (line.empty() || line[0] == '#')
			continue;

		if (line.rfind("BEGIN_POLYGON", 0) == 0 || line.rfind("POLYGON", 0) == 0) {
			currentEdges.clear();
			inPolygon = true;
			continue;
		}

		if (line.rfind("END_POLYGON", 0) == 0 || line.rfind("END", 0) == 0) {
			if (inPolygon && !currentEdges.empty()) {
				PolygonData pd;
				pd.edges = std::move(currentEdges);
				out.push_back(std::move(pd));
				currentEdges.clear();
			}
			inPolygon = false;
			continue;
		}

		if (inPolygon) {
			auto tokens = SplitLine(line);
			PolygonEdge edge;
			if (ParseEdge(tokens, edge))
				currentEdges.push_back(edge);
		}
	}

	// 处理文件末尾没有 END_POLYGON 的情况
	if (inPolygon && !currentEdges.empty()) {
		PolygonData pd;
		pd.edges = std::move(currentEdges);
		out.push_back(std::move(pd));
	}

	return !out.empty();
}

/**
 * 将多边形列表写入文件
 */
bool ExportPolygons(const std::string& filepath, const std::vector<PolygonData>& polygons) {
	std::ofstream file(filepath);
	if (!file.is_open()) {
		std::cerr << "  [ERROR] Cannot write file: " << filepath << std::endl;
		return false;
	}

	file << "# Polygon Data File\n";
	file << "# Format: startX startY endX endY bulge\n";
	file << "# bulge: 0 = line, non-zero = arc\n";
	file << "\n";

	for (size_t pi = 0; pi < polygons.size(); ++pi) {
		file << "BEGIN_POLYGON " << (pi + 1) << "\n";
		for (const auto& e : polygons[pi].edges) {
			file << e.start.x << " " << e.start.y << " "
				<< e.end.x << " " << e.end.y << " "
				<< e.bulge << "\n";
		}
		file << "END_POLYGON\n\n";
	}

	return true;
}

// ============================================================================
//  将 PolygonEdge 转换为 tailor 使用的弧线段类型（支持 bulge）
//  bulge = 0  → 直线段（LineSegment）
//  bulge ≠ 0  → 弧线段（ArcSegment）
// ============================================================================

using Arc = tailor::ArcSegment<Point2d, double>;

/** 将 PolygonEdge 转为 Arc（兼容线段和弧线） */
static std::vector<Arc> ToArcs(const PolygonData& poly) {
	std::vector<Arc> arcs;
	arcs.reserve(poly.edges.size());
	for (const auto& e : poly.edges)
		arcs.emplace_back(e.start, e.end, e.bulge);
	return arcs;
}

// ============================================================================
//  测试函数
// ============================================================================

using IntersectionPattern = tailor::IntersectionPattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;
using UnionPattern = tailor::UnionPattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;
using DifferencePattern = tailor::DifferencePattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;
using SymmetricDifferencePattern = tailor::SymmetricDifferencePattern<tailor::EvenOddFillType, tailor::EvenOddFillType>;

/** 使用指定精度 N 对 A、B 两组多边形执行交/并/差/补布尔运算并输出结果 */
template<int N>
static void RunBooleanOps(const std::string& tag,
	const std::vector<PolygonData>& polysA,
	const std::vector<PolygonData>& polysB) {
	using ArcAnalyserP = tailor::ArcAnalysis<Arc, tailor::ArcSegmentAnalyserCore<Arc, tailor::PrecisionCore<N>>>;
	using TailorTypeP = tailor::Tailor<Arc, ArcAnalyserP>;

	ArcAnalyserP analyser;
	TailorTypeP tailor(analyser);

	// 将 A 组多边形加入 PolygonSetA
	for (const auto& poly : polysA) {
		auto arcs = ToArcs(poly);
		tailor.AddToPolygonSetA(arcs.begin(), arcs.end());
	}

	// 将 B 组多边形加入 PolygonSetB
	for (const auto& poly : polysB) {
		auto arcs = ToArcs(poly);
		tailor.AddToPolygonSetB(arcs.begin(), arcs.end());
	}

	std::cout << "  [precision=" << N << "] A=" << polysA.size() << " B=" << polysB.size() << std::endl;
	auto drafting = tailor.Execute();
	std::cout << "  Edge events: " << drafting.edgeEvent.size()
		<< ", Vertex groups: " << drafting.vertexEvents.size() << std::endl;

	// 序列化 drafting 到文件
	tailor::polygon_io::WriteDraftingFile<Arc>("D://drafting.txt", drafting.edgeEvent, drafting.vertexEvents);

	{
		auto polys = IntersectionPattern().Stitch(drafting);
		std::cout << "    Intersection       : " << polys.size() << " result polygon(s)" << std::endl;
	}
	{
		auto polys = UnionPattern().Stitch(drafting);
		std::cout << "    Union              : " << polys.size() << " result polygon(s)" << std::endl;
	}
	{
		auto polys = DifferencePattern().Stitch(drafting);
		std::cout << "    Difference         : " << polys.size() << " result polygon(s)" << std::endl;
	}
	{
		auto polys = SymmetricDifferencePattern().Stitch(drafting);
		std::cout << "    SymmetricDifference: " << polys.size() << " result polygon(s)" << std::endl;
	}

	std::cout << "  [" << tag << "] OK" << std::endl;
}

// ============================================================================
//  主入口
// ============================================================================

/** 探测数据目录: 依次尝试可能的路径前缀 */
static std::string FindDataDir() {
	const std::vector<std::string> candidates = {
		"test/test_data/",           // 从项目根目录启动
		"../test/test_data/",        // 从 build/ 目录启动
		"../../test/test_data/",     // 从 build/test/Debug 等子目录启动
	};
	for (const auto& d : candidates) {
		std::ifstream testFile(d + "error_poly.txt");
		if (testFile.is_open())
			return d;
	}
	return "test/test_data/";  // fallback
}

int main() {
	std::cout << "========================================" << std::endl;
	std::cout << "  Tailor Polygon Data Tests" << std::endl;
	std::cout << "  (交/并/差/补)" << std::endl;
	std::cout << "========================================" << std::endl;

	const std::string dataDir = FindDataDir();
	std::cout << "Data directory: " << dataDir << std::endl;

	std::vector<std::string> files = {
		//"error_poly.txt",   "error_poly2.txt",  "error_poly3.txt",
		//"error_poly4.txt",  "error_poly5.txt",  "error_poly6.txt",
		//"error_poly7.txt",  "error_poly8.txt",  "error_poly9.txt",
		//"error_poly10.txt", "error_poly11.txt", "error_poly12.txt",
		//"error_poly13.txt", "error_poly14.txt",
		"error_poly15.txt",
	};

	int failed = 0;

	// 精度尝试顺序: 10 → 9 → 11
	constexpr std::array<int, 3> precisionOrder = { 9,10,  11 };

	for (const auto& f : files) {
		std::string path = dataDir + f;
		std::cout << "\n--- " << f << " ---" << std::endl;

		// 先加载多边形数据（与精度无关）
		std::vector<PolygonData> polys;
		if (!ImportPolygons(path, polys)) {
			std::cout << "  SKIP (cannot read)\n";
			continue;
		}

		std::cout << "  Loaded " << polys.size() << " polygon(s)" << std::endl;

		// 交替分配: 偶数索引 → A, 奇数索引 → B
		std::vector<PolygonData> polysA, polysB;
		for (size_t i = 0; i < polys.size(); ++i) {
			if (i % 2 == 0)
				polysA.push_back(std::move(polys[i]));
			else
				polysB.push_back(std::move(polys[i]));
		}

		// 依次尝试各精度，成功则跳出
		bool ok = false;
		for (int p : precisionOrder) {
			try {
				switch (p) {
				case 9:  RunBooleanOps<9>(f, polysA, polysB);  break;
				case 11: RunBooleanOps<11>(f, polysA, polysB); break;
				case 10: RunBooleanOps<10>(f, polysA, polysB); break;
				}
				ok = true;
				break;
			}
			catch (const std::exception& e) {
				std::cout << "  [precision=" << p << "] failed: " << e.what() << std::endl;
			}
			catch (...) {
				std::cout << "  [precision=" << p << "] unknown exception" << std::endl;
			}
		}

		if (!ok) {
			std::cerr << "  [FAIL] " << f << ": all precisions failed" << std::endl;
			++failed;
		}
	}

	std::cout << "\n========================================" << std::endl;
	std::cout << "  Results: " << (files.size() - failed) << "/" << files.size()
		<< " files passed" << std::endl;
	if (failed > 0)
		std::cout << "  " << failed << " FAILED" << std::endl;
	std::cout << "========================================" << std::endl;

	return failed;
}