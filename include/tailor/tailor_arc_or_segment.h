#include "tailor_segment.h"
#include <numbers>
#include <array>
#include <vector>

TAILOR_NAMESPACE_BEGIN
// TODO 规范常量命名

// C++20?
// TODO 避免全大写
constexpr auto TAILOR_PI = std::numbers::pi;
constexpr auto TAILOR_2PI = 2 * std::numbers::pi;
constexpr auto TAILOR_PI2 = std::numbers::pi / 2.0;
constexpr auto TAILOR_PI4 = std::numbers::pi / 4.0;

template<class PType, class T, class UserData = void>
class ArcSegment :public LineSegment<PType, UserData> {
	T bulge = 0;
public:
	using PointType = PType;
	using UserDataType = UserData;

	template<class... Args>
	constexpr ArcSegment(const PointType& a, const PointType& b, const T& bulge, Args&&... args) :
		LineSegment<PType, UserData>(a, b, std::forward<Args>(args)...), bulge(bulge) {
	}

	T& Bulge() { return bulge; }
	const T& Bulge() const { return bulge; }
};

using ArcOrSeg = ArcSegment<Point<double>, double>;

template<class ArcType>
class ArcSegmentTraits {
public:
	using PointType = typename ArcType::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;
	using CurveType = ArcType;
	using UserDataType = typename CurveType::UserDataType;
public:
	static const PointType& Point0(const CurveType& curve) { return curve.Point0(); }
	static const PointType& Point1(const CurveType& curve) { return curve.Point1(); }

	// 此版本的曲线是支持直线的, 所以在计算圆心和半径及顺时针方向前, 必须调用此函数
	static bool IsArc(const CurveType& curve) {
		using namespace std;
		return abs(curve.Bulge()) > 0;// tolerance ???
	}

	// https://blog.csdn.net/axin620/article/details/148099679
	PointType Center(const CurveType& curve) const {
		assert(IsArc(curve));

		auto bluge = curve.Bulge();
		auto b = 0.5 * (1 / bluge - bluge);

		auto x1 = pUtils.X(Point0(curve));
		auto x2 = pUtils.X(Point1(curve));
		auto y1 = pUtils.Y(Point0(curve));
		auto y2 = pUtils.Y(Point1(curve));

		auto x = 0.5 * (x1 + x2 - b * (y2 - y1));
		auto y = 0.5 * (y1 + y2 + b * (x2 - x1));
		return pUtils.ConstructPoint(x, y);
	}

	CoordinateType Radius(const CurveType& curve) const {
		assert(IsArc(curve));

		auto vec = pUtils.Sub(Point1(curve), Point0(curve));
		auto g = pUtils.Len(vec);

		auto bluge = curve.Bulge();
		using std::abs;
		return abs(0.25 * g * (1 / bluge + bluge));
	}

	static bool CCW(const CurveType& curve) {
		return curve.Bulge() > 0; // tolerance ???
	}

	// 圆弧构造
	CurveType Construct(const PointType& a, const PointType& b, const CurveType& from) const {
		if (!IsArc(from)) {
			if constexpr (std::is_same_v<typename CurveType::UserDataType, void>) {
				return CurveType(a, b, 0);
			} else {
				return CurveType(a, b, 0, from.Data());
			}
		}
		const auto o = Center(from);
		const auto oa = pUtils.Sub(a, o);
		const auto ob = pUtils.Sub(b, o);

		using namespace std;
		auto ta = atan2(pUtils.Y(oa), pUtils.X(oa));
		if (ta < 0) ta += TAILOR_2PI;
		auto tb = atan2(pUtils.Y(ob), pUtils.X(ob));
		if (tb < 0) tb += TAILOR_2PI;

		if (CCW(from)) {
			if (tb < ta) tb += TAILOR_2PI;
		} else {
			if (ta < tb) ta += TAILOR_2PI;
		}

		using std::tan;
		auto bluge = tan((tb - ta) / 4.0);

		if constexpr (std::is_same_v<typename CurveType::UserDataType, void>) {
			return CurveType(a, b, bluge);
		} else {
			return CurveType(a, b, bluge, from.Data());
		}
	}
private:
	using PUtils = PointUtils<PointType>;
	PUtils pUtils;
};

template<class ArcType>
class ArcSegmentUtils {
private:
	using PointType = typename ArcType::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;
	using CurveType = ArcType;
	using UserDataType = typename CurveType::UserDataType;

public:
	struct Intersections {
		std::optional<PointType> points[2];
		bool isCoincident = false;
	};

	// 只判断完整曲线的相交, 交点是否在曲线上, 需要调用者自行判断
	Intersections Intersect(const CurveType& ab, const CurveType& cd, CoordinateType tolerance) const {
		bool ab_is_arc = cTraits.IsArc(ab);
		bool cd_is_arc = cTraits.IsArc(cd);

		if (ab_is_arc && cd_is_arc) {
			return IntersectCircleCircle(ab, cd, tolerance);
		} else if (!ab_is_arc && cd_is_arc) {
			return IntersectLineCircle(ab, cd, tolerance);
		} else if (ab_is_arc && !cd_is_arc) {
			return IntersectCircleLine(ab, cd, tolerance);
		} else {
			throw "no impl";
		}
	}

	Intersections IntersectLineCircle(const CurveType& ab, const CurveType& cd, CoordinateType tolerance) const {
		auto cd_center = cTraits.Center(cd);
		auto foot = ProjectLine(cd_center, ab);
		auto cd_radius = cTraits.Radius(cd);

		double distance = pUtils.Len(pUtils.Sub(cd_center, foot));

		Intersections res{};

		if (distance > cd_radius + tolerance) {
			// 直线与圆相离
			return res;
		}

		if (std::fabs(distance - cd_radius) < tolerance) {
			// 直线与圆相切
			res.points[0] = foot;
			return res;
		}

		double half_chord = std::sqrt(cd_radius * cd_radius - distance * distance);

		auto ab_dir = pUtils.Normlize(pUtils.Sub(cTraits.Point0(ab), cTraits.Point1(ab)));
		auto vec = pUtils.Mult(ab_dir, half_chord);
		// 两个交点
		res.points[0] = pUtils.Add(foot, vec);
		res.points[1] = pUtils.Sub(foot, vec);

		return res;
	}

	Intersections IntersectCircleLine(const CurveType& ab, const CurveType& cd, CoordinateType tolerance) const {
		return IntersectLineCircle(cd, ab, tolerance);
	}

	Intersections IntersectLineLine(const CurveType& ab, const CurveType& cd, CoordinateType tolerance) const {
		throw "no impl";
		return {};
	}

	Intersections IntersectCircleCircle(const CurveType& ab, const CurveType& cd, CoordinateType tolerance) const {
		auto ab_center = cTraits.Center(ab);
		auto cd_center = cTraits.Center(cd);
		auto ab_radius = cTraits.Radius(ab);
		auto cd_radius = cTraits.Radius(cd);
		Intersections res{};

		if (pUtils.IsSamePosition(ab_center, cd_center, tolerance) && std::fabs(ab_radius - cd_radius) < tolerance) {
			// 圆弧可能重合
			res.isCoincident = true;
			return res;
		}

		auto oo = pUtils.Sub(cd_center, ab_center);
		auto oo_len = pUtils.Len(oo);
		if (oo_len > (ab_radius + cd_radius)) {
			// 两圆相离
			return res;
		}
		if (oo_len < std::fabs(ab_radius - cd_radius)) {
			// 内含
			return res;
		}

		if (std::fabs(oo_len - (ab_radius + cd_radius)) < tolerance) {
			// 两圆外切
			auto u = std::atan2(pUtils.Y(oo), pUtils.X(oo));
			auto p = pUtils.Add(ab_center,
				pUtils.ConstructPoint(
					ab_radius * std::cos(u), ab_radius * std::sin(u)
				)
			);
			res.points[0] = p;
			return res;
		}

		if (std::abs(oo_len - std::abs(ab_radius - cd_radius)) < tolerance) {
			// 内切
			double ratio = ab_radius / oo_len;
			if (ab_radius > cd_radius) {
				auto p = pUtils.ConstructPoint(
					ab_center.x + ratio * oo.x,
					ab_center.y + ratio * oo.y
				);
				res.points[0] = p;
			} else {
				auto p = pUtils.ConstructPoint(
					ab_center.x - ratio * oo.x,
					ab_center.y - ratio * oo.y
				);
				res.points[0] = p;
			}
			return res;
		}

		double a = (ab_radius * ab_radius - cd_radius * cd_radius + oo_len * oo_len) / (2.0 * oo_len);
		double h = std::sqrt(ab_radius * ab_radius - a * a);

		// 中点
		double x2 = ab_center.x + (a * oo.x) / oo_len;
		double y2 = ab_center.y + (a * oo.y) / oo_len;

		// 计算两个交点
		double rx = -oo.y * (h / oo_len);
		double ry = +oo.x * (h / oo_len);

		res.points[0] = pUtils.ConstructPoint(x2 + rx, y2 + ry);
		res.points[1] = pUtils.ConstructPoint(x2 - rx, y2 - ry);
		return res;
	}

	CurveType ConstructCurve(const PointType& a, const PointType& b, const CurveType& from) const {
		return cTraits.Construct(a, b, from);
	}
private:
	bool IsSamePosition(const PointType& a, const PointType& b) const {
		return pUtils.IsSamePosition(a, b, 1e-10);
	}

	PointType ProjectLine(const PointType& p, const CurveType& line) const {
		decltype(auto) p0 = cTraits.Point0(line);
		decltype(auto) p1 = cTraits.Point1(line);
		const auto u = ProjectUWithoutClamp(p, p0, p1);
		auto v = static_cast<CoordinateType>(1) - u;
		return PointType(v * pUtils.X(p0) + u * pUtils.X(p1), v * pUtils.Y(p0) + u * pUtils.Y(p1));
	}

	CoordinateType ProjectUWithoutClamp(const PointType& p, const PointType& a, const PointType& b) const {
		auto ab_x = pUtils.X(b) - pUtils.X(a);
		auto ab_y = pUtils.Y(b) - pUtils.Y(a);
		auto ap_x = pUtils.X(p) - pUtils.X(a);
		auto ap_y = pUtils.Y(p) - pUtils.Y(a);
		auto v = ab_x * ap_x + ab_y * ap_y;

		auto length_sq = ab_x * ab_x + ab_y * ab_y;
		assert(length_sq > 0);
		return v / length_sq;
	}
private:
	using PUtils = PointUtils<PointType>;
	using CurveTraits = ArcSegmentTraits<CurveType>;

	CurveTraits cTraits;
	PUtils pUtils;
};

// 偏特化
// 当ArcSegment表现为Segment时, 调用LineSegmentUtils时, 会用到此特化
template <class PType, class T, class UserData>
struct LineSegmentTraits<ArcSegment<PType, T, UserData>> {
	using CurveType = ArcSegment<PType, T, UserData>;
	using PointType = typename CurveType::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;

	static constexpr PointType GetPoint0(const CurveType& point) { return point.Point0(); }
	static constexpr PointType GetPoint1(const CurveType& point) { return point.Point1(); }

	static constexpr CurveType Construct(const PointType& p0, const PointType& p1, const CurveType& from) {
		if constexpr (std::is_same_v<typename CurveType::UserDataType, void>) {
			return CurveType(p0, p1, 0);
		} else {
			return CurveType(p0, p1, 0, from.Data());
		}
	}
};

template<class ArcSegmentType, class PC>
class ArcSegmentAnalyserCore :
	public PointUtils<typename ArcSegmentTraits<ArcSegmentType>::PointType>,
	public ArcSegmentTraits<ArcSegmentType>,
	public ArcSegmentUtils<ArcSegmentType> {
	using Super0 = PointUtils<typename ArcSegmentTraits<ArcSegmentType>::PointType>;
	using Super1 = ArcSegmentTraits<ArcSegmentType>;
	using Super2 = ArcSegmentUtils<ArcSegmentType>;
public:
	using Precision = PC;
public:
	ArcSegmentAnalyserCore() = default;
	template<class... Args>
	auto DirectionFromTo(Args&&... args) const {
		return Super0::DirectionFromTo(std::forward<Args>(args)..., Precision::VALUE_EPSILON);
	}

	template<class... Args>
	auto Intersect(Args&&... args) const {
		return Super2::Intersect(std::forward<Args>(args)..., Precision::VALUE_EPSILON);
	}

	template<class... Args>
	auto IsSamePosition(Args&&... args) const {
		return Super0::IsSamePosition(std::forward<Args>(args)..., Precision::VALUE_EPSILON);
	}
};

template<class ArcType, class Core>
class ArcAnalysis {
public:
	using PointType = typename ArcType::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;
	using CurveType = ArcType;

	static const PointType& Start(const CurveType& curve) { return curve.Point0(); }
	static const PointType& End(const CurveType& curve) { return curve.Point1(); }
	static CurveType Reverse(const CurveType& curve) {
		using namespace std;
		auto copy = curve;
		swap(copy.Point0(), copy.Point1());
		copy.Bulge() = -copy.Bulge();
		return copy;
	}
	VertexRelativePositionType CalcateVertexRelativePosition(const PointType& a, const PointType& b) const {
		return core.DirectionFromTo(a, b);
	}

	CurveRelativePositionResult2<CurveType> CalcateEdgeRelativePosition(const CurveType& ab, const CurveType& cd,
		tailor::OnlyFocusOnRelativePositionAndCoincidenceDetails) const {
		// 如果都是直线，调用contour.h中的直线段版本
		if (!core.IsArc(ab) && !core.IsArc(cd)) {
			using Segment = CurveType;
			using PC = typename Core::Precision;
			using LineCore = tailor::LineSegmentAnalyserCore<Segment, PC>;
			auto line_analyzer = tailor::LineSegmentAnalyser<Segment, LineCore>{};
			auto line_res = line_analyzer.CalcateEdgeRelativePosition(ab, cd, tailor::OnlyFocusOnRelativePositionAndCoincidenceDetails{});
			return line_res;
		}

		const auto& A = core.Point0(ab);
		const auto& C = core.Point0(cd);
		assert(CalcateVertexRelativePosition(A, C) == tailor::VertexRelativePositionType::Same);

		auto tan_A = Tangent(A, ab);
		auto tan_C = Tangent(C, cd);

		CurveRelativePositionResult2<CurveType> res;
		// 如果切线方向相同, 则认为重合
		// XXX 这里应该使用向量算法, 而非点算法
		auto tan_vrp = CalcateVertexRelativePosition(tan_A, tan_C);
		if (tan_vrp == tailor::VertexRelativePositionType::Same) {
			// 重合 或 相切
			if (core.IsArc(ab) && core.IsArc(cd)) {
				auto center_ab = core.Center(ab);
				auto center_cd = core.Center(cd);
				auto center_vrp = CalcateVertexRelativePosition(center_ab, center_cd);

				constexpr auto Positive = [](VertexRelativePositionType type) {
					return static_cast<int>(type) > 0;
					};

				// 重合
				if (center_vrp == tailor::VertexRelativePositionType::Same) {
					const auto& B = core.Point1(ab);
					const auto& D = core.Point1(cd);
					auto type = CalcateVertexRelativePosition(B, D);
					res.positionType = CurveRelativePositionType::Coincident;
					if (type == tailor::VertexRelativePositionType::Same) {
						// 完全重合
						res.edges[static_cast<size_t>(PieceType::AI)] = ab;
						res.edges[static_cast<size_t>(PieceType::CI)] = cd;
						return res;
					} else {
						if (Positive(type)) {
							// B 在 cd 上
							auto split_res = SplitEdge(cd, B);
							res.edges[static_cast<size_t>(PieceType::AI)] = ab;
							res.edges[static_cast<size_t>(PieceType::CI)] = split_res.GetPiece(AI);
							res.edges[static_cast<size_t>(PieceType::ID)] = split_res.GetPiece(IB);
							return res;
						} else {
							// D 在 ab 上
							auto split_res = SplitEdge(ab, D);
							res.edges[static_cast<size_t>(PieceType::CI)] = cd;
							res.edges[static_cast<size_t>(PieceType::AI)] = split_res.GetPiece(AI);
							res.edges[static_cast<size_t>(PieceType::IB)] = split_res.GetPiece(IB);
							return res;
						}
					}
				}

				const auto& B = core.Point1(ab);
				const auto& D = core.Point1(cd);
				auto end_vrp = CalcateVertexRelativePosition(B, D);

				// 相切
				if (Positive(end_vrp)) {
					// B 点在前面, 采样 B 点
					auto y_in_ab = SampleInX(ab, B);
					auto y_in_cd = SampleInX(cd, B);
					// C 点不可能在 ab 上, 这种情况前面已经判断过了
					res.positionType = (y_in_ab > y_in_cd) ?
						CurveRelativePositionType::Downward :
						CurveRelativePositionType::Upward;
					return res;
				} else {
					// D 点在前面, 采样 D 点
					auto y_in_ab = SampleInX(ab, D);
					auto y_in_cd = SampleInX(cd, D);
					// C 点不可能在 ab 上, 这种情况前面已经判断过了
					res.positionType = (y_in_ab > y_in_cd) ?
						CurveRelativePositionType::Downward :
						CurveRelativePositionType::Upward;
					return res;
				}
			}

			// 一个为圆弧, 一个为直线,
			// 相切
			if (core.IsArc(ab)) {
				if (core.CCW(ab)) {
					res.positionType = CurveRelativePositionType::Downward;
				} else {
					res.positionType = CurveRelativePositionType::Upward;
				}
			} else {
				if (core.CCW(cd)) {
					res.positionType = CurveRelativePositionType::Upward;
				} else {
					res.positionType = CurveRelativePositionType::Downward;
				}
			}
			return res;
		}

		if (tan_vrp == tailor::VertexRelativePositionType::Top ||
			tan_vrp == tailor::VertexRelativePositionType::LeftTop ||
			tan_vrp == tailor::VertexRelativePositionType::RightTop) {
			// cd 在 ab 上方
			res.positionType = CurveRelativePositionType::Upward;
		} else {
			// cd 在 ab 下方
			res.positionType = CurveRelativePositionType::Downward;
		}
		return res;
	}

	CurveRelativePositionResult2<CurveType> CalcateEdgeRelativePosition(const CurveType& ab, const CurveType& cd,
		tailor::OnlyFocusOnRelativePositionWithoutCoincidence) const {
		// 如果都是直线，调用contour.h中的直线段版本
		if (!core.IsArc(ab) && !core.IsArc(cd)) {
			using Segment = CurveType;
			using PC = typename Core::Precision;
			using LineCore = tailor::LineSegmentAnalyserCore<Segment, PC>;
			auto line_analyzer = tailor::LineSegmentAnalyser<Segment, LineCore>{};
			auto line_res = line_analyzer.CalcateEdgeRelativePosition(ab, cd, tailor::OnlyFocusOnRelativePositionWithoutCoincidence{});
			return line_res;
		}

		// 包含圆弧的情况，使用与模板函数相同的逻辑
		CurveRelativePositionResult2<CurveType> res;

		const auto& A = core.Point0(ab);
		const auto& C = core.Point0(cd);

		if (CalcateVertexRelativePosition(A, C) == tailor::VertexRelativePositionType::Same) {
			// 调用 OnlyFocusOnRelativePositionAndCoincidenceDetails 版本, 但理论上重合分支不会进
			return CalcateEdgeRelativePosition(ab, cd, tailor::OnlyFocusOnRelativePositionAndCoincidenceDetails{});
		}

		// 采样 C 点
		auto y_in_ab = SampleInX(ab, C);
		auto y_in_cd = SampleInX(cd, C);

		// C 点不可能在 ab 上, 这种情况前面已经判断过了
		res.positionType = (y_in_ab > y_in_cd) ?
			CurveRelativePositionType::Downward :
			CurveRelativePositionType::Upward;

		return res;
	}

	CurveRelativePositionResult2<CurveType> CalcateEdgeRelativePosition(const CurveType& ab, const CurveType& cd,
		tailor::OnlyFocusOnRelativePositionDetailsWithoutCoincidence) const {
		// 如果都是直线，调用contour.h中的直线段版本
		if (!core.IsArc(ab) && !core.IsArc(cd)) {
			using Segment = CurveType;
			using PC = typename Core::Precision;
			using LineCore = tailor::LineSegmentAnalyserCore<Segment, PC>;
			auto line_analyzer = tailor::LineSegmentAnalyser<Segment, LineCore>{};
			auto line_res = line_analyzer.CalcateEdgeRelativePosition(ab, cd, tailor::OnlyFocusOnRelativePositionDetailsWithoutCoincidence{});
			return line_res;
		}

		// 包含圆弧的情况，使用与模板函数相同的逻辑
		CurveRelativePositionResult2<CurveType> res;

		auto inters = core.Intersect(ab, cd);

		for (size_t i = 0; i < 2; ++i) {
			auto& pnt = inters.points[i];
			if (!pnt.has_value()) break;
			if (core.IsSamePosition(pnt.value(), ab.Point0())) {
				pnt = std::nullopt;// 与A点重合, 忽略
				continue;
			}
			// 交点必须在两条曲线上
			if (!IsOnEdge(pnt.value(), ab) || !IsOnEdge(pnt.value(), cd)) {
				pnt = std::nullopt;// 与A点重合, 忽略
				continue;
			}
		}

		const auto& A = ab.Point0();
		const auto& B = ab.Point1();
		const auto& C = cd.Point0();
		const auto& D = cd.Point1();

		auto sample_pnt = CalcSamplePoint(C, B, D);
		auto y_in_ab = SampleInX(ab, sample_pnt);
		auto y_in_cd = SampleInX(cd, sample_pnt);

		res.positionType = (y_in_ab > y_in_cd) ?
			CurveRelativePositionType::Downward :
			CurveRelativePositionType::Upward;

		if (!inters.points[0].has_value() && !inters.points[1].has_value()) {
			// 未相交
			res.edges[static_cast<size_t>(PieceType::AI)] = ab;
			res.edges[static_cast<size_t>(PieceType::CI)] = cd;
			return res;
		}

		PointType I{};
		if (inters.points[0].has_value() && inters.points[1].has_value()) {
			// 选取前面的一个交点
			auto vrp = CalcateVertexRelativePosition(inters.points[0].value(), inters.points[1].value());
			I = static_cast<int>(vrp) > 0 ?
				inters.points[0].value() :
				inters.points[1].value();
		} else if (inters.points[0].has_value() && !inters.points[1].has_value()) {
			I = inters.points[0].value();
		} else {
			I = inters.points[1].value();
		}

		res.edges[static_cast<size_t>(PieceType::AI)] = core.Construct(A, I, ab);
		res.edges[static_cast<size_t>(PieceType::CI)] = core.Construct(C, I, cd);
		if (!core.IsSamePosition(I, B)) {
			res.edges[static_cast<size_t>(PieceType::IB)] = core.Construct(I, B, ab);
		}
		if (!core.IsSamePosition(I, D)) {
			res.edges[static_cast<size_t>(PieceType::ID)] = core.Construct(I, D, cd);
		}
		return res;
	}

	PointType Tangent(const PointType& p, const CurveType& curve) const {
		assert(IsOnEdge(p, curve));

		if (!core.IsArc(curve)) {
			auto dir = core.Sub(curve.Point1(), curve.Point0());
			return core.Normlize(dir);
		} else {
			auto o = core.Center(curve);
			auto op = core.Sub(p, o);
			auto r = core.Radius(curve);
			auto tangent_dir = core.Mult(core.ConstructPoint(-core.Y(op), core.X(op)), 1.0 / r);
			if (!core.CCW(curve)) {
				tangent_dir = core.Mult(tangent_dir, -1);
			}
			return tangent_dir;
		}
	}

	bool IsOnEdge(const PointType& p, const CurveType& arc) const {
		if (!core.IsArc(arc)) {
			// TODO
			// 直线段
			using Segment = CurveType;
			using PC = typename Core::Precision;
			using LineCore = tailor::LineSegmentAnalyserCore<Segment, PC>;
			auto line_analyzer = tailor::LineSegmentAnalyser<Segment, LineCore>{};
			return line_analyzer.IsOnEdge(p, arc);
		}

		auto o = core.Center(arc);
		auto r = core.Radius(arc);

		auto op = core.Sub(p, o);
		auto l = core.Len(op);

		using std::abs;
		if (abs(r - l) > 1e-7) {
			return false;
		}

		auto a = core.Point0(arc);
		auto b = core.Point1(arc);
		const auto oa = core.Sub(a, o);
		const auto ob = core.Sub(b, o);

		using std::atan2;
		auto ta = atan2(core.Y(oa), core.X(oa));
		if (ta < 0) ta += TAILOR_2PI;
		auto tb = atan2(core.Y(ob), core.X(ob));
		if (tb < 0) tb += TAILOR_2PI;

		auto tp = atan2(core.Y(op), core.X(op));
		if (tp < 0) tp += TAILOR_2PI;

		if (core.CCW(arc)) {
			if (tb < ta) tb += TAILOR_2PI;
			if (tp < ta) tp += TAILOR_2PI;

			return tp <= tb;
		} else {
			if (ta < tb) tb -= TAILOR_2PI;
			if (ta < tp) tp -= TAILOR_2PI;

			return tp >= tb;
		}
	}

	SplitEdgeResult<CurveType> SplitEdge(const CurveType& curve, const PointType& p) const {
		return {
			core.Construct(core.Point0(curve), p, curve),
			core.Construct(p, core.Point1(curve), curve)
		};
	}

	// ERROR
	MonotonicSplitResult<CurveType> SplitToMonotonic(const CurveType& edge) {
		if (!ArcSegmentUtils<ArcType>{}.IsArc(edge)) {
			return MonotonicSplitResult<CurveType>{edge};
		}

		const auto a = edge.Point0();
		const auto b = edge.Point1();
		const auto o = core.Center(edge);

		const auto oa = core.Sub(a, o);
		const auto ob = core.Sub(b, o);
		{
			auto r0 = core.Radius(edge);
			auto r1 = core.Len(oa);
			auto r2 = core.Len(ob);
		}

		using namespace std;
		auto ta = atan2(core.Y(oa), core.X(oa));
		if (ta < 0) ta += TAILOR_2PI;
		auto tb = atan2(core.Y(ob), core.X(ob));
		if (tb < 0) tb += TAILOR_2PI;
	}

	// TODO
	std::vector<CurveType> SplitToMonotonic2(const CurveType& edge) const {
		if (!ArcSegmentTraits<ArcType>{}.IsArc(edge)) {
			return { edge };
		}

		const auto a = core.Point0(edge);
		const auto b = core.Point1(edge);
		const auto o = core.Center(edge);

		const auto oa = core.Sub(a, o);
		const auto ob = core.Sub(b, o);
		{
			auto r0 = core.Radius(edge);
			auto r1 = core.Len(oa);
			auto r2 = core.Len(ob);
		}

		using std::atan2;
		auto ta = atan2(core.Y(oa), core.X(oa));
		if (ta < 0) ta += TAILOR_2PI;
		auto tb = atan2(core.Y(ob), core.X(ob));
		if (tb < 0) tb += TAILOR_2PI;

		std::vector<double> ts;
		ts.reserve(5);
		constexpr auto region = TAILOR_PI;

		if (core.CCW(edge)) {
			if (tb < ta) tb += TAILOR_2PI;

			using std::floor;
			auto t = floor(ta / region) + 1;
			for (;; ++t) {
				auto angle = t * region;
				if (tb <= angle) break;
				ts.emplace_back(angle);
			}
		} else {
			if (ta < tb) ta += TAILOR_2PI;

			using std::ceil;
			auto t = ceil(ta / region) - 1;
			for (;; --t) {
				auto angle = t * region;
				if (tb >= angle) break;
				ts.emplace_back(angle);
			}
		}

		if (ts.empty()) {
			return { edge };
		}

		std::vector<PointType> points;
		points.emplace_back(a);
		auto r = core.Radius(edge);
		for (auto t : ts) {
			points.emplace_back(core.ConstructPoint(o.x + r * cos(t), o.y + r * sin(t)));
		}
		points.emplace_back(b);

		std::vector<CurveType> res;
		for (size_t i = 1, n = points.size(); i < n; ++i) {
			// TODO 如果两个点过于接近, 则跳过

			res.emplace_back(
				core.ConstructCurve(points[i - 1], points[i], edge)
			);
		}
		return res;
	}

private:
	PointType CalcSamplePoint(const PointType& C, const PointType& B, const PointType& D) const {
		auto vrp = CalcateVertexRelativePosition(B, D);
		if (static_cast<int>(vrp) > 0) {
			return core.Divide(core.Add(C, B), 2);
		} else {
			return core.Divide(core.Add(C, D), 2);
		}
	}

	double SampleInX(const CurveType& ab, const PointType& p) const {
		auto a = ab.Point0();
		auto b = ab.Point1();
		if (!core.IsArc(ab)) {
			auto ab_vec = core.Sub(ab.Point1(), ab.Point0());
			return core.Y(a) + core.Y(ab_vec) / core.X(ab_vec) * (core.X(p) - core.X(a));
		}

		auto center = core.Center(ab);
		auto r = core.Radius(ab);
		auto d = core.X(p) - core.X(center);

		using namespace std;
		if (abs(d - r) <= 1e-7 || abs(d + r) <= 1e-7) {
			// 误差
			// 在 C 点处采样, 但 C 点为单调分割点, 半径计算误差导致 d 略长于 r
			return core.Y(center);
		} else if (d - r >= 1e-7 /*采样点在圆右边*/ || d + r <= -1e-7 /*采样点在圆左边*/) {
			throw "";
		}

		auto l = std::sqrt(r * r - d * d);

		if (core.CCW(ab)) {
			return core.Y(center) - l;
		} else {
			return core.Y(center) + l;
		}
	}

private:
	Core core{};
};

TAILOR_NAMESPACE_END