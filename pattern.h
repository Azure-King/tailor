#pragma once
#include "tailor_concept.h"
#include <assert.h>
#include <vector>
#include <ranges>

TAILOR_NAMESPACE_BEGIN

struct EdgeFillStatus {
	Int wind = 0;	 // 边(下方)的环绕数
	Size positive = 0; // 正方向边的数量
	Size negitive = 0; // 负方向边的数量
};

struct EdgeGroupFillStatus {
	EdgeFillStatus subject;
	EdgeFillStatus clipper;
};

enum class BoundaryType :short {
	Unknown = 0b0000,			// 未知

	UpperBoundary = 0b0001,     // 上边界
	LowerBoundary = 0b0010,	    // 下边界
	ConjugateBoundary = UpperBoundary | LowerBoundary,	// 共轭边界, 同时表示上下边界, 仅特殊情况使用

	Inside = 0b0100,			// 内部
	Outside = 0b1000,			// 外部

	InsideConjugateBoundary = Inside | ConjugateBoundary,	// 内部共轭边界
	OutsideConjugateBoundary = Outside | ConjugateBoundary,	// 外部共轭边界
};

namespace {
	inline bool IsBoundary(BoundaryType type) {
		return type == BoundaryType::UpperBoundary
			|| type == BoundaryType::LowerBoundary;
	}
	inline bool IsBoundaryX(BoundaryType type) {
		return type == BoundaryType::UpperBoundary
			|| type == BoundaryType::LowerBoundary
			|| type == BoundaryType::InsideConjugateBoundary
			|| type == BoundaryType::OutsideConjugateBoundary;
		//return (static_cast<int>(type) & 0b0011) != 0;
	}
	constexpr BoundaryType RemoveUpperBoundary(BoundaryType type) {
		return static_cast<BoundaryType>(static_cast<int>(type) &
			static_cast<int>(BoundaryType::LowerBoundary));
	}

	constexpr BoundaryType RemoveLowerBoundary(BoundaryType type) {
		return static_cast<BoundaryType>(static_cast<int>(type) &
			static_cast<int>(BoundaryType::UpperBoundary));
	}

	constexpr bool HasLowerBoundary(BoundaryType type) {
		return (static_cast<int>(type) & static_cast<int>(BoundaryType::LowerBoundary)) != 0;
	}
	constexpr bool HasUpperBoundary(BoundaryType type) {
		return (static_cast<int>(type) & static_cast<int>(BoundaryType::UpperBoundary)) != 0;
	}

	inline BoundaryType ReverseBoundary(BoundaryType type) {
		assert(IsBoundary(type));
		return (type == BoundaryType::UpperBoundary) ?
			BoundaryType::LowerBoundary :
			BoundaryType::UpperBoundary;
	}
}

// 等于指定环绕数
template<Int wind>
class EqSpecifiedWindCondition {
public:
	constexpr bool operator()(Int w) const {
		static_assert(wind != 0, "Zero winding number = outside the polygon.");
		return w == wind;
	}
};

// 不等于指定环绕数
template<Int wind>
class NeqSpecifiedWindCondition {
public:
	constexpr bool operator()(Int w) const {
		return w != wind && w != 0;
	}
};

// 大于等于指定环绕数
template<Int wind>
class GeqSpecifiedWindCondition {
public:
	constexpr bool operator()(Int w) const {
		return w != 0 && w >= wind;
	}
};

// 小于等于指定环绕数
template<Int wind>
class LeqSpecifiedWindCondition {
public:
	constexpr bool operator()(Int w) const {
		return w != 0 && w <= wind;
	}
};

// 小于指定环绕数
template<Int wind>
class LtSpecifiedWindCondition {
public:
	constexpr bool operator()(Int w) const {
		return w != 0 && w < wind;
	}
};

// 大于指定环绕数
template<Int wind>
class GtSpecifiedWindCondition {
public:
	constexpr bool operator()(Int w) const {
		return w != 0 && w > wind;
	}
};

// 奇偶条件
class EvenOddCondition {
public:
	constexpr bool operator()(Int wind) const {
		return wind % 2 != 0;
	}
};

// 非零条件
class NonZeroCondition {
public:
	constexpr bool operator()(Int wind) const {
		return wind != 0;
	}
};

// 永假条件
class UnsatisfiableCondition {
public:
	constexpr bool operator()(Int wind) const {
		return false;
	}
};

// 正
// Condition = bool(Int); 返回值表示是否在多边形内部, Func(0) 必须为 false
template<class Condition>
class ConditionFillType {
public:
	template<class... Args>
	ConditionFillType(Args&&... args) :condition(std::forward<Args>(args)...) {
		// condition(0) 必须为 false, tailor 规定外部环绕从 0 开始
		assert(!condition(0));
	}

	BoundaryType operator()(const EdgeFillStatus& status) const {
		auto x = static_cast<Int>(status.positive) - static_cast<Int>(status.negitive);

		bool succ0 = condition(status.wind);
		bool succ1 = condition(status.wind + x);

		if (succ0 && succ1) {
			//  w+x 内
			//<----->
			//   w  内
			return BoundaryType::Inside;
		} else if (succ0) {
			//  w+x 外
			//<-----
			//   w  内
			return BoundaryType::UpperBoundary;
		} else if (succ1) {
			//  w+x 内
			// ----->
			//   w  外
			return BoundaryType::LowerBoundary;
		} else {
			//  w+x 外
			//<----->
			//   w  外
			return BoundaryType::Outside;
		}
	}
private:
	[[no_unique_address]]
	Condition condition;
};

// 支持共轭边界的版本
template<class Condition>
class ConditionFillType2 {
public:
	template<class... Args>
	ConditionFillType2(Args&&... args) :condition(std::forward<Args>(args)...) {
		// condition(0) 必须为 false, tailor 规定外部环绕从 0 开始
		assert(!condition(0));
	}

	BoundaryType operator()(const EdgeFillStatus& status) const {
		auto x = static_cast<Int>(status.positive) - static_cast<Int>(status.negitive);
		bool succ0 = condition(status.wind);
		bool succ1 = condition(status.wind + x);

		if (succ0 && succ1) {
			bool conjugate =
				status.positive + status.negitive != 0 &&     // 该边属于该组
				(status.positive + status.negitive) % 2 == 0; // 该边成对出现

			//  w+x 内
			//<----->
			//   w  内
			return conjugate ? BoundaryType::InsideConjugateBoundary : BoundaryType::Inside;
		} else if (succ0) {
			//  w+x 外
			//<-----
			//   w  内
			return BoundaryType::UpperBoundary;
		} else if (succ1) {
			//  w+x 内
			// ----->
			//   w  外
			return BoundaryType::LowerBoundary;
		} else {
			bool conjugate =
				status.positive + status.negitive != 0 &&
				(status.positive + status.negitive) % 2 == 0;

			//  w+x 外
			//<----->
			//   w  外
			return conjugate ? BoundaryType::OutsideConjugateBoundary : BoundaryType::Outside;
		}
	}
private:
	[[no_unique_address]]
	Condition condition;
};

using EvenOddFillType = ConditionFillType<EvenOddCondition>;
using NonZeroFillType = ConditionFillType<NonZeroCondition>;
using IgnoreFillType = ConditionFillType<UnsatisfiableCondition>;

struct PolyEdgeInfo {
	Handle id = npos;
	BoundaryType type = BoundaryType::Unknown; // 上边界或下边界, 但如果是上边界, 则该边需要反转
};

template<class Edge>
struct Polygon {
	std::vector<Edge> edges;
};

template<class Edge>
struct PolyTree {
	using PolygonType = Polygon<Edge>;

	PolygonType polygon;
	std::vector<PolyTree<Edge>> children;
};

/**
 * @brief 并集 subject | clipper
 */
class UnionOperation {
public:
	BoundaryType operator()(BoundaryType subject_status, BoundaryType clipper_status) const {
		if (IsBoundary(subject_status) && IsBoundary(clipper_status)) {
			return (subject_status == clipper_status) ? subject_status : BoundaryType::Inside;
		} else if (IsBoundary(subject_status)) {
			return (clipper_status == BoundaryType::Outside) ? subject_status : BoundaryType::Inside;
		} else if (IsBoundary(clipper_status)) {
			return (subject_status == BoundaryType::Outside) ? clipper_status : BoundaryType::Inside;
		} else {
			return (subject_status == BoundaryType::Inside
				|| clipper_status == BoundaryType::Inside) ?
				BoundaryType::Inside : BoundaryType::Outside;
		}
	}
};

// 0 <- Inside
// 1 <- Outside
// 2 <- UpperBoundary
// 3 <- LowerBoundary
// 4 <- InsideConjugateBoundary
// 5 <- OutsideConjugateBoundary
class BoundaryTypeIndexMap {
	using enum BoundaryType;
	static constexpr Size invalidIndex = static_cast<Size>(-1);

	static constexpr std::array<Size, 12> indexMap{
		invalidIndex,2/*UpperBoundary*/,3/*LowerBoundary*/,invalidIndex,
		0/*Inside*/,invalidIndex,invalidIndex,4/*InsideConjugateBoundary*/,
		1/*Outside*/,invalidIndex,invalidIndex,5/*OutsideConjugateBoundary*/
	};

public:
	static constexpr Size Index(BoundaryType type) {
		assert(static_cast<Size>(type) < 12);
		assert(indexMap[static_cast<Size>(type)] != invalidIndex);

		return indexMap[static_cast<Size>(type)];
	}
};

class UnionXOperation {
	using enum BoundaryType;
	// | A                       | B                       | Result                  | Or                     |
	// | Inside                  | Inside                  | Inside                  |                        |
	// | Inside                  | Outside                 | Inside                  |                        |
	// | Inside                  | UpperBoundary           | Inside                  |                        |
	// | Inside                  | LowerBoundary           | Inside                  |                        |
	// | Inside                  | InsideConjugateBoundary | Inside                  |                        |
	// | Inside                  | OutsideConjugateBoundary| Inside                  |                        |
	// | Outside                 | Inside                  | Inside                  |                        |
	// | Outside                 | Outside                 | Outside                 |                        |
	// | Outside                 | UpperBoundary           | UpperBoundary           |                        |
	// | Outside                 | LowerBoundary           | LowerBoundary           |                        |
	// | Outside                 | InsideConjugateBoundary | InsideConjugateBoundary |                        |
	// | Outside                 | OutsideConjugateBoundary| OutsideConjugateBoundary|                        |
	// | UpperBoundary           | Inside                  | Inside                  |                        |
	// | UpperBoundary           | Outside                 | UpperBoundary           |                        |
	// | UpperBoundary           | UpperBoundary           | UpperBoundary           |                        |
	// | UpperBoundary           | LowerBoundary           | Inside                  | InsideConjugateBoundary|
	// | UpperBoundary           | InsideConjugateBoundary | InsideConjugateBoundary | Inside                 | 
	// | UpperBoundary           | OutsideConjugateBoundary| UpperBoundary           |                        |
	// | LowerBoundary           | Inside                  | Inside                  |                        |
	// | LowerBoundary           | Outside                 | LowerBoundary           |                        |
	// | LowerBoundary           | UpperBoundary           | Inside                  | InsideConjugateBoundary|
	// | LowerBoundary           | LowerBoundary           | LowerBoundary           |                        |
	// | LowerBoundary           | InsideConjugateBoundary | InsideConjugateBoundary | Inside                 |
	// | LowerBoundary           | OutsideConjugateBoundary| LowerBoundary           |                        |
	// | InsideConjugateBoundary | Inside                  | Inside                  |                        |
	// | InsideConjugateBoundary | Outside                 | InsideConjugateBoundary |                        |
	// | InsideConjugateBoundary | UpperBoundary           | InsideConjugateBoundary | Inside                 |
	// | InsideConjugateBoundary | LowerBoundary           | InsideConjugateBoundary | Inside                 |
	// | InsideConjugateBoundary | InsideConjugateBoundary | InsideConjugateBoundary |                        |
	// | InsideConjugateBoundary | OutsideConjugateBoundary| Inside                  |                        |
	// | OutsideConjugateBoundary| Inside                  | Inside                  |                        |
	// | OutsideConjugateBoundary| Outside                 | OutsideConjugateBoundary|                        |
	// | OutsideConjugateBoundary| UpperBoundary           | UpperBoundary           |                        |
	// | OutsideConjugateBoundary| LowerBoundary           | LowerBoundary           |                        |
	// | OutsideConjugateBoundary| InsideConjugateBoundary | Inside                  |                        |
	// | OutsideConjugateBoundary| OutsideConjugateBoundary| OutsideConjugateBoundary|                        |
	static constexpr std::array<BoundaryType, 36> unionMap{
		Inside,Inside,Inside,Inside,Inside,Inside,
		Inside,Outside,UpperBoundary,LowerBoundary,InsideConjugateBoundary,OutsideConjugateBoundary,
		Inside,UpperBoundary,UpperBoundary,Inside,InsideConjugateBoundary,UpperBoundary,
		Inside,LowerBoundary,Inside,LowerBoundary,InsideConjugateBoundary,LowerBoundary,
		Inside,InsideConjugateBoundary,InsideConjugateBoundary,InsideConjugateBoundary,InsideConjugateBoundary,Inside,
		Inside,OutsideConjugateBoundary,UpperBoundary,LowerBoundary,Inside,OutsideConjugateBoundary,
	};
public:
	BoundaryType operator()(BoundaryType subject_status, BoundaryType clipper_status) const {
		// 还是打表快 XD
		return unionMap[
			BoundaryTypeIndexMap::Index(subject_status) * 6 + BoundaryTypeIndexMap::Index(clipper_status)
		];
	}
};

/**
 * @brief 差集 subject - clipper
 */
class DifferenceOperation {
public:
	BoundaryType operator()(BoundaryType subject_status, BoundaryType clipper_status) const {
		if (IsBoundary(subject_status) && IsBoundary(clipper_status)) {
			return (subject_status != clipper_status) ? subject_status : BoundaryType::Outside;
		} else if (IsBoundary(subject_status)) {
			return (clipper_status == BoundaryType::Outside) ? subject_status : BoundaryType::Outside;
		} else if (IsBoundary(clipper_status)) {
			return (subject_status == BoundaryType::Inside) ? ReverseBoundary(clipper_status) : BoundaryType::Outside;
		} else {
			return (subject_status == BoundaryType::Outside
				|| clipper_status == BoundaryType::Inside) ?
				BoundaryType::Outside : BoundaryType::Inside;
		}
	}
};

/**
 * @brief 反向差集 clipper - subject
 */
class ReverseDifferenceOperation {
public:
	BoundaryType operator()(BoundaryType subject_status, BoundaryType clipper_status) const {
		return DifferenceOperation()(clipper_status, subject_status);
	}
};

/**
 * @brief 异或(XOR) subject ^ clipper
 */
class SymmetricDifferenceOperation {
public:
	BoundaryType operator()(BoundaryType subject_status, BoundaryType clipper_status) const {
		auto res0 = DifferenceOperation()(clipper_status, subject_status);
		if (IsBoundary(res0)) return res0;

		auto res1 = DifferenceOperation()(subject_status, clipper_status);
		if (IsBoundary(res1)) return res1;

		return (res0 == BoundaryType::Inside || res1 == BoundaryType::Inside) ?
			BoundaryType::Inside : BoundaryType::Outside;
	}
};

/**
 * @brief 交集 subject & clipper
 */
class IntersectionOperation {
public:
	BoundaryType operator()(BoundaryType subject_status, BoundaryType clipper_status) const {
		if (IsBoundary(subject_status) && IsBoundary(clipper_status)) {
			return (subject_status == clipper_status) ? subject_status : BoundaryType::Outside;
		} else if (IsBoundary(subject_status)) {
			return (clipper_status == BoundaryType::Inside) ? subject_status : BoundaryType::Outside;
		} else if (IsBoundary(clipper_status)) {
			return (subject_status == BoundaryType::Inside) ? clipper_status : BoundaryType::Outside;
		} else {
			return (subject_status == BoundaryType::Outside
				|| clipper_status == BoundaryType::Outside) ?
				BoundaryType::Outside : BoundaryType::Inside;
		}
	}
};

// 如果优先连接外边界
class ConnectTypeOuterFirst {
public:
	/**
	 * @brief	连接所有边界边为多边形
	 * @tparam Drafting	草稿类型
	 * @param drafting	草稿
	 * @param types		边界类型数组
	 * @return	多边形集合
	 */
	template<class Drafting>
	std::vector<Polygon<PolyEdgeInfo>> Connect(const Drafting& drafting, std::vector<BoundaryType> types) const {
		const auto& vertices = drafting.vertexEvents;
		const auto& edges = drafting.edgeEvent;

		std::vector<Polygon<PolyEdgeInfo>> polys;
		for (size_t i = 0, n = edges.size(); i < n; ++i) {
			if (!IsBoundary(types[i])) continue;

			auto& poly = polys.emplace_back();

			Handle first = i;
			Handle current = i;
			Handle next = tailor::npos;

			do {
				assert(tailor::npos != current);

				auto edge_id = current;
				const auto& edge = edges[edge_id];
				auto curr_edge_type = types[edge_id];

				assert(IsBoundary(curr_edge_type));

				// 必须为上边界或下边界
				if (BoundaryType::LowerBoundary == curr_edge_type) {
					// 如果是下边界
					// 则本条边的终点和下条边的起点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.endPntGroup];

					auto span0 = vertex.endGroup.Span();
					auto span1 = vertex.startGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;

					// 此处顺时针找
					{
						std::span<const Handle> group = span0.last(span0.size() - index - 1);
						for (auto e : group) {
							if (types[e] == BoundaryType::UpperBoundary) {
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : std::ranges::reverse_view(span1)) {
							if (types[e] == BoundaryType::LowerBoundary) { // 方向相同, 边界属性也相同
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.first(index);
						for (auto e : group) {
							if (types[e] == BoundaryType::UpperBoundary) {
								target = e;
								break;
							}
						}
					}

					next = target;
				} else {
					// 如果是上边界
					// 则本条边的起点和下条边的终点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.startPntGroup];

					auto span0 = vertex.startGroup.Span();
					auto span1 = vertex.endGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;

					// 此处顺时针找
					{
						std::span<const Handle> group = span0.first(index);
						for (auto e : std::ranges::reverse_view(group)) {
							if (types[e] == BoundaryType::LowerBoundary) {
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : span1) {
							if (types[e] == BoundaryType::UpperBoundary) { // 方向相同, 边界属性也相同
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.last(span0.size() - index - 1);
						for (auto e : std::ranges::reverse_view(group)) {
							if (types[e] == BoundaryType::LowerBoundary) {
								target = e;
								break;
							}
						}
					}

					next = target;
				}

				assert(tailor::npos != next);

				poly.edges.push_back({ current, curr_edge_type });

				current = next;
				next = tailor::npos;
			} while (current != first);

			// 重置所有已处理边的类型
			for (auto& e : poly.edges) {
				types[e.id] = BoundaryType::Unknown;
			}
		}

		return polys;
	}
};

class ConnectTypeInnerFirst {
public:
	/**
	 * @brief	连接所有边界边为多边形
	 * @tparam Drafting	草稿类型
	 * @param drafting	草稿
	 * @param types		边界类型数组
	 * @return	多边形集合
	 */
	template<class Drafting>
	std::vector<Polygon<PolyEdgeInfo>> Connect(const Drafting& drafting, std::vector<BoundaryType> types) const {
		const auto& vertices = drafting.vertexEvents;
		const auto& edges = drafting.edgeEvent;

		std::vector<Polygon<PolyEdgeInfo>> polys;
		for (size_t i = 0, n = edges.size(); i < n; ++i) {
			if (!IsBoundary(types[i])) continue;

			auto& poly = polys.emplace_back();

			Handle first = i;
			Handle current = i;
			Handle next = tailor::npos;

			do {
				assert(tailor::npos != current);

				auto edge_id = current;
				const auto& edge = edges[edge_id];
				auto curr_edge_type = types[edge_id];

				assert(IsBoundary(curr_edge_type));

				// 必须为上边界或下边界
				if (BoundaryType::LowerBoundary == curr_edge_type) {
					// 如果是下边界
					// 则本条边的终点和下条边的起点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.endPntGroup];

					auto span0 = vertex.endGroup.Span();
					auto span1 = vertex.startGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;

					// 此处逆时针找
					{
						std::span<const Handle> group = span0.first(index);
						for (auto e : std::ranges::reverse_view(group)) {
							if (types[e] == BoundaryType::UpperBoundary) {
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : span1) {
							if (types[e] == BoundaryType::LowerBoundary) { // 方向相同, 边界属性也相同
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.last(span0.size() - index - 1);

						for (auto e : std::ranges::reverse_view(group)) {
							if (types[e] == BoundaryType::UpperBoundary) {
								target = e;
								break;
							}
						}
					}

					next = target;
				} else {
					// 如果是上边界
					// 则本条边的起点和下条边的终点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.startPntGroup];

					auto span0 = vertex.startGroup.Span();
					auto span1 = vertex.endGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;

					// 此处顺时针找
					{
						std::span<const Handle> group = span0.last(span0.size() - index - 1);
						for (auto e : group) {
							if (types[e] == BoundaryType::LowerBoundary) {
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : std::ranges::reverse_view(span1)) {
							if (types[e] == BoundaryType::UpperBoundary) { // 方向相同, 边界属性也相同
								target = e;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.first(index);
						for (auto e : group) {
							if (types[e] == BoundaryType::LowerBoundary) {
								target = e;
								break;
							}
						}
					}

					next = target;
				}

				assert(tailor::npos != next);

				poly.edges.push_back({ current, curr_edge_type });

				current = next;
				next = tailor::npos;
			} while (current != first);

			// 重置所有已处理边的类型
			for (auto& e : poly.edges) {
				types[e.id] = BoundaryType::Unknown;
			}
		}

		return polys;
	}
};

class OuterFirstXConnectType {
public:
	/**
	 * @brief	连接所有边界边为多边形
	 * @tparam Drafting	草稿类型
	 * @param drafting	草稿
	 * @param types		边界类型数组
	 * @return	多边形集合
	 */
	template<class Drafting>
	std::vector<Polygon<PolyEdgeInfo>> Connect(const Drafting& drafting, std::vector<BoundaryType> types) const {
		const auto& vertices = drafting.vertexEvents;
		const auto& edges = drafting.edgeEvent;

		std::vector<Polygon<PolyEdgeInfo>> polys;
		for (size_t i = 0, n = edges.size(); i < n; ++i) {
			if (!IsBoundaryX(types[i])) continue;

			auto& poly = polys.emplace_back();

			Handle first = i;
			Handle current = i;
			BoundaryType first_boundary = HasLowerBoundary(types[current]) ?
				BoundaryType::LowerBoundary : BoundaryType::UpperBoundary;
			BoundaryType curr_boundary = first_boundary;
			Handle next = tailor::npos;
			BoundaryType next_boundary = BoundaryType::Unknown;

			do {
				assert(tailor::npos != current);

				auto edge_id = current;
				const auto& edge = edges[edge_id];
				assert(IsBoundaryX(curr_boundary));

				// 必须为上边界或下边界
				if (HasLowerBoundary(curr_boundary)) {
					// 如果是下边界
					// 则本条边的终点和下条边的起点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.endPntGroup];

					auto span0 = vertex.endGroup.Span();
					auto span1 = vertex.startGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;
					BoundaryType target_boundary = BoundaryType::Unknown;

					// 此处顺时针找第一个上边界
					{
						//std::span<const Handle> group = span0.last(span0.size() - index - 1);
						std::span<const Handle> group = span0.last(span0.size() - index);
						for (auto e : group) {
							if (HasUpperBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::UpperBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						//
						for (auto e : std::ranges::reverse_view(span1)) {
							if (HasLowerBoundary(types[e])) { // 方向相同, 边界属性也相同
								target = e;
								target_boundary = BoundaryType::LowerBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.first(index);
						for (auto e : group) {
							if (HasUpperBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::UpperBoundary;
								break;
							}
						}
					}

					next = target;
					next_boundary = target_boundary;
				} else {
					assert(HasUpperBoundary(curr_boundary));

					// 如果是上边界
					// 则本条边的起点和下条边的终点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.startPntGroup];

					auto span0 = vertex.startGroup.Span();
					auto span1 = vertex.endGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;
					BoundaryType target_boundary = BoundaryType::Unknown;

					// 此处顺时针找
					{
						std::span<const Handle> group = span0.first(index + 1);
						for (auto e : std::ranges::reverse_view(group)) {
							if (HasLowerBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::LowerBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : span1) {
							if (HasUpperBoundary(types[e])) { // 方向相同, 边界属性也相同
								target = e;
								target_boundary = BoundaryType::UpperBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.last(span0.size() - index - 1);
						for (auto e : std::ranges::reverse_view(group)) {
							if (HasLowerBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::LowerBoundary;
								break;
							}
						}
					}

					next = target;
					next_boundary = target_boundary;
				}

				assert(tailor::npos != next);

				poly.edges.push_back({ current, curr_boundary });

				current = next;
				curr_boundary = next_boundary;

				next = tailor::npos;
				next_boundary = BoundaryType::Unknown;
			} while (current != first || (current == first && curr_boundary != first_boundary));

			// 重置所有已处理边的类型
			for (auto& e : poly.edges) {
				assert(
					e.type == BoundaryType::UpperBoundary ||
					e.type == BoundaryType::LowerBoundary
				);

				if (e.type == BoundaryType::UpperBoundary) {
					types[e.id] = RemoveUpperBoundary(types[e.id]);
				} else {
					types[e.id] = RemoveLowerBoundary(types[e.id]);
				}
			}
		}

		return polys;
	}
};

class InnerFirstXConnectType {
public:
	/**
	 * @brief	连接所有边界边为多边形
	 * @tparam Drafting	草稿类型
	 * @param drafting	草稿
	 * @param types		边界类型数组
	 * @return	多边形集合
	 */
	template<class Drafting>
	std::vector<Polygon<PolyEdgeInfo>> Connect(const Drafting& drafting, std::vector<BoundaryType> types) const {
		const auto& vertices = drafting.vertexEvents;
		const auto& edges = drafting.edgeEvent;

		std::vector<Polygon<PolyEdgeInfo>> polys;
		for (size_t i = 0, n = edges.size(); i < n; ++i) {
			if (!IsBoundaryX(types[i])) continue;

			auto& poly = polys.emplace_back();

			Handle first = i;
			Handle current = i;
			BoundaryType first_boundary = HasLowerBoundary(types[current]) ?
				BoundaryType::LowerBoundary : BoundaryType::UpperBoundary;
			BoundaryType curr_boundary = first_boundary;
			Handle next = tailor::npos;
			BoundaryType next_boundary = BoundaryType::Unknown;

			do {
				assert(tailor::npos != current);

				auto edge_id = current;
				const auto& edge = edges[edge_id];
				assert(IsBoundaryX(curr_boundary));

				// 必须为上边界或下边界
				if (HasLowerBoundary(curr_boundary)) {
					// 如果是下边界
					// 则本条边的终点和下条边的起点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.endPntGroup];

					auto span0 = vertex.endGroup.Span();
					auto span1 = vertex.startGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;
					BoundaryType target_boundary = BoundaryType::Unknown;

					// 此处顺时针找
					{
						std::span<const Handle> group = span0.first(index);
						for (auto e : std::ranges::reverse_view(group)) {
							if (HasUpperBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::UpperBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : span1) {
							if (HasLowerBoundary(types[e])) { // 方向相同, 边界属性也相同
								target = e;
								target_boundary = BoundaryType::LowerBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.last(span0.size() - index);
						for (auto e : std::ranges::reverse_view(group)) {
							if (HasUpperBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::UpperBoundary;
								break;
							}
						}
					}

					next = target;
					next_boundary = target_boundary;
				} else {
					assert(HasUpperBoundary(curr_boundary));

					// 如果是上边界
					// 则本条边的起点和下条边的终点是同一个顶点(所有边始终为正向)
					auto& vertex = vertices[edge.startPntGroup];

					auto span0 = vertex.startGroup.Span();
					auto span1 = vertex.endGroup.Span();
					auto loc = std::find(span0.begin(), span0.end(), edge_id);
					// 不可能找不到
					assert(loc != span0.end());

					std::size_t index = std::distance(span0.begin(), loc);

					Handle target = tailor::npos;
					BoundaryType target_boundary = BoundaryType::Unknown;

					// 此处顺时针找第一个上边界
					{
						//std::span<const Handle> group = span0.last(span0.size() - index - 1);
						std::span<const Handle> group = span0.last(span0.size() - index - 1);
						for (auto e : group) {
							if (HasLowerBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::LowerBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						// 必须反着遍历
						for (auto e : std::ranges::reverse_view(span1)) {
							if (HasUpperBoundary(types[e])) { // 方向相同, 边界属性也相同
								target = e;
								target_boundary = BoundaryType::UpperBoundary;
								break;
							}
						}
					}

					if (tailor::npos == target) {
						std::span<const Handle> group = span0.first(index + 1);
						for (auto e : group) {
							if (HasLowerBoundary(types[e])) {
								target = e;
								target_boundary = BoundaryType::LowerBoundary;
								break;
							}
						}
					}

					next = target;
					next_boundary = target_boundary;
				}

				assert(tailor::npos != next);

				poly.edges.push_back({ current, curr_boundary });

				current = next;
				curr_boundary = next_boundary;

				next = tailor::npos;
				next_boundary = BoundaryType::Unknown;
			} while (current != first || (current == first && curr_boundary != first_boundary));

			// 重置所有已处理边的类型
			for (auto& e : poly.edges) {
				assert(
					e.type == BoundaryType::UpperBoundary ||
					e.type == BoundaryType::LowerBoundary
				);

				if (e.type == BoundaryType::UpperBoundary) {
					types[e.id] = RemoveUpperBoundary(types[e.id]);
				} else {
					types[e.id] = RemoveLowerBoundary(types[e.id]);
				}
			}
		}

		return polys;
	}
};

/**
 * @brief  普通布尔运算模式
 * @tparam SubjectFillType		subject 填充类型
 * @tparam ClipperFillType		clipper 填充类型
 * @tparam BoolOperationType	布尔运算类型
 */
template<class SubjectFillType, class ClipperFillType,
	class ConnectType, class BoolOperationType>
class OrdinaryBoolOperationPattern {
public:
	[[no_unique_address]]
	SubjectFillType subjectFillType;
	[[no_unique_address]]
	ClipperFillType clipperFillType;
	[[no_unique_address]]
	ConnectType connectType;
	[[no_unique_address]]
	BoolOperationType boolOperationType;
public:
	OrdinaryBoolOperationPattern() = default;

	template<class SFT, class CFT, class CT, class BOT>
	OrdinaryBoolOperationPattern(SFT&& sft, CFT&& cft, CT&& ct, BOT&& bt) :
		subjectFillType(std::forward<SFT>(sft)),
		clipperFillType(std::forward<CFT>(cft)),
		connectType(std::forward<CT>(ct)),
		boolOperationType(std::forward<BOT>(bt)) {
	}

private:
	template<class EdgeEvent>
	void AddSingleEdgeStatus(const EdgeEvent& edge, EdgeGroupFillStatus& status) const {
		auto& fs = edge.isClipper ? status.clipper : status.subject;
		edge.reversed ? (++fs.negitive) : (++fs.positive);
	}

	template<class Drafting>
	EdgeGroupFillStatus CalcEdgeFillStatus(const Drafting& drafting, const typename Drafting::EdgeEvent& edge) const {
		EdgeGroupFillStatus res{};
		res.clipper.wind = edge.clipperWind;
		res.subject.wind = edge.subjectWind;

		if (!edge.aggregatedEdges) {
			AddSingleEdgeStatus(edge, res);
			return res;
		}

		for (auto id : edge.aggregatedEdges->sourceEdges) {
			AddSingleEdgeStatus(drafting.edgeEvent[id], res);
		}
		return res;
	}

	template<class Drafting>
	PolyEdgeInfo FirstBottomEdge(const Drafting& drafting, const PolyEdgeInfo& edge, const std::vector<BoundaryType>& types) const {
		auto raw_type = types[edge.id];
		assert(IsBoundaryX(raw_type));
		const auto& edges = drafting.edgeEvent;

		if (raw_type == BoundaryType::OutsideConjugateBoundary) {
			if (edge.type == BoundaryType::UpperBoundary) {
				return { edge.id, BoundaryType::LowerBoundary };
			}
		} else if (raw_type == BoundaryType::InsideConjugateBoundary) {
			if (edge.type == BoundaryType::LowerBoundary) {
				return { edge.id, BoundaryType::UpperBoundary };
			}
		}

		Handle first = edges[edge.id].firstBottom;
		if (tailor::npos == first) return {}; // 下方无边

		while (tailor::npos != edges[first].firstSplit) {
			first = edges[first].firstSplit;
		}
		if (tailor::npos == first) return {}; // 下方无边

		auto next_raw_type = types[first];
		if (next_raw_type == BoundaryType::OutsideConjugateBoundary) {
			return { first, BoundaryType::UpperBoundary };
		} else if (next_raw_type == BoundaryType::InsideConjugateBoundary) {
			return { first, BoundaryType::LowerBoundary };
		}
		return { first, next_raw_type };
	}

	/**
	 * @brief 获取多边形中的第一个下边界边
	 * @tparam 草稿类型
	 * @param drafting 草稿
	 * @param polygon  多边形
	 * @param types	   类型表
	 * @return
	 */
	template<class Drafting>
	PolyEdgeInfo LowestEdge(const Drafting& drafting, const Polygon<PolyEdgeInfo>& polygon, const std::vector<BoundaryType>& types) const {
		const auto& edges = drafting.edgeEvent;
		const auto& vertices = drafting.vertexEvents;

		PolyEdgeInfo result{};
		Handle vertex_id = tailor::npos;
		for (const auto& edge_info : polygon.edges) {
			auto cur_vertex_id = edges[edge_info.id].startPntGroup;
			if (cur_vertex_id > vertex_id) continue;

			if (cur_vertex_id == vertex_id) {
				if (edge_info.id == result.id) {
					// 共轭边
					assert(
						types[edge_info.id] == BoundaryType::InsideConjugateBoundary ||
						types[edge_info.id] == BoundaryType::OutsideConjugateBoundary
					);

					if (types[edge_info.id] == BoundaryType::InsideConjugateBoundary) {
						// ----->
						// InsideConjugateBoundary 顺时针旋转
						// <-----
						result.type = BoundaryType::UpperBoundary;
					} else {
						// <-----
						// OutsideConjugateBoundary 逆时针旋转
						// ----->
						result.type = BoundaryType::LowerBoundary;
					}
					continue;
				}

				auto span = vertices[cur_vertex_id].startGroup.Span();
				bool no_change = false;
				for (auto eid : span) {
					if (eid == result.id) {
						no_change = true;
						break;
					}
					if (eid == edge_info.id) {
						no_change = false;
						break;
					}
				}
				if (no_change) continue;
			}

			// 更新
			vertex_id = cur_vertex_id;
			result = edge_info;
		}

		assert(tailor::npos != result.id);
		assert(tailor::npos != vertex_id);

		return result;
	}

public:

	template<class Drafting>
	auto Stitch(Drafting& drafting) const {
		auto& vertices = drafting.vertexEvents;
		auto& edges = drafting.edgeEvent;
		size_t size = edges.size();
		std::vector<BoundaryType> types(size);

		for (const auto& edge : edges) {
			if (!edge.end) continue;

			EdgeGroupFillStatus status = CalcEdgeFillStatus<Drafting>(drafting, edge);

			auto subject_boundary_status = subjectFillType(status.subject);
			auto clipper_boundary_status = clipperFillType(status.clipper);
			auto res_boundary_status = boolOperationType(
				subject_boundary_status, clipper_boundary_status
			);

			types[edge.id] = res_boundary_status;
		}

		std::vector<BoundaryType> types2(types);

		std::vector<Polygon<PolyEdgeInfo>> polys = connectType.Connect(drafting, types);

		// 禁用树结构（暂时）
		if (false) {
			std::vector<PolyTree<PolyEdgeInfo>> trees;
			return trees;
		}

		static constexpr size_t invalid_group = static_cast<size_t>(-1);
		constexpr auto CalcId = [](const PolyEdgeInfo& edge)->size_t {
			assert(IsBoundary(edge.type));
			return edge.id * 2 + static_cast<size_t>(edge.type == BoundaryType::LowerBoundary);
			};

		// 每组输出多边形的边对应的组
		std::vector<size_t> groups(edges.size() * 2, invalid_group);
		for (size_t i = 0, n = polys.size(); i < n; ++i) {
			for (auto& edge : polys[i].edges) {
				groups[CalcId(edge)] = i;
			}
		}

		struct GroupInfo {
			size_t parent = invalid_group;
			size_t brother = invalid_group; // 当 brother 为 npos 时, 规定 parent 计算完毕

			bool isOuter = false;
			PolyEdgeInfo lowestEdgeInfo{};
		};

		std::vector<GroupInfo> group_infos(polys.size());

		// 计算每组的最低边和边界属性
		for (size_t i = 0, n = group_infos.size(); i < n; ++i) {
			auto& group_info = group_infos[i];

			// 如果最低边是下边界, 则为外边界
			group_info.lowestEdgeInfo = LowestEdge(drafting, polys[i], types2);
			assert(IsBoundary(group_info.lowestEdgeInfo.type));
			group_info.isOuter = HasLowerBoundary(group_info.lowestEdgeInfo.type);
		}

		// 计算每组之间的关系
		for (size_t i = 0, n = group_infos.size(); i < n; ++i) {
			auto& group_info = group_infos[i];

			PolyEdgeInfo fbe = FirstBottomEdge(
				drafting, group_info.lowestEdgeInfo, types
			);

			// 有可能找到非输出边, 直到
			while (
				tailor::npos != fbe.id &&                  // 下方有边
				IsBoundary(fbe.type) &&                    // 必须为合法的边界边
				invalid_group == groups[CalcId(fbe)]) {
				fbe = FirstBottomEdge(drafting, fbe, types);
			}

			// 本组为独立的外边界
			if (tailor::npos == fbe.id || !IsBoundary(fbe.type)) continue;

			// 由于 flbe 已经是最低的了, 所以不会找到依旧为本组的更低的边
			auto group_id = groups[CalcId(fbe)];
			assert(i != group_id);
			assert(invalid_group != group_id);
			assert(IsBoundaryX(types2[fbe.id]));

			if (group_infos[group_id].isOuter == group_info.isOuter) {
				// i 和 group_id 同级
				group_info.brother = group_id;
			} else {
				// i 是 group_id 下级
				group_info.parent = group_id;
			}
		}
		// 计算每组的父节点
		for (size_t i = 0, n = group_infos.size(); i < n; ++i) {
			auto& group_info = group_infos[i];
			if (invalid_group == group_info.brother) continue;

			size_t brother = group_info.brother;
			while (true) {
				auto& brother_info = group_infos[brother];
				if (invalid_group == brother_info.brother) {
					group_info.parent = brother_info.parent;
					break;
				}
				brother = brother_info.brother;
			}
		}

		// 构建树
		std::vector<size_t> indegree(group_infos.size());
		// 构建入度表
		for (size_t i = 0, n = group_infos.size(); i < n; ++i) {
			auto& group_info = group_infos[i];
			if (invalid_group == group_info.parent) {
				assert(group_info.isOuter); // 最外层一定是外环
				continue; // 为根节点
			}
			indegree[group_info.parent]++;
		}

		std::vector<PolyTree<PolyEdgeInfo>> trees(group_infos.size());
		for (size_t i = 0, n = trees.size(); i < n; ++i) {
			trees[i].polygon = std::move(polys[i]);
		}

		// 将具有父节点的节点移入父节点中
		// 循环次数和层数有关
		bool success = false;
		while (!success) {
			success = true;
			for (size_t i = 0, n = indegree.size(); i < n; ++i) {
				if (0 == indegree[i]) {
					if (invalid_group == group_infos[i].parent) {
						// 如果为根节点, 说明改节点已经准备完毕
					} else {
						trees[group_infos[i].parent].children.emplace_back(std::move(trees[i]));
						indegree[group_infos[i].parent]--;
						indegree[i] = invalid_group;// 设定非法值, 表示已经
						success = false;
					}
				}
			}
		}

		// 移除无用的节点
		trees.erase(std::remove_if(trees.begin(), trees.end(),
			[](const PolyTree<PolyEdgeInfo>& tree) {
				return tree.polygon.edges.empty();
			}), trees.end());

		return trees;
	}
};

#define SIMPLE_PATTERN_DEF(OPERATION) template<class SubjectFillType, class ClipperFillType, \
	class ConnectType = ConnectTypeOuterFirst> \
class OPERATION##Pattern : public OrdinaryBoolOperationPattern< \
	SubjectFillType, ClipperFillType, \
ConnectType, OPERATION##Operation\
> {\
public:\
		OPERATION##Pattern() = default; \
template<class SFT, class CFT, class CT>\
OPERATION##Pattern(SFT&& sft, CFT&& cft, CT&& ct) :\
OrdinaryBoolOperationPattern<\
	SubjectFillType, ClipperFillType, \
	ConnectType, OPERATION##Operation \
>(std::forward<SFT>(sft), std::forward<CFT>(cft), \
	std::forward<CT>(ct), OPERATION##Operation{}) {\
}\
};

SIMPLE_PATTERN_DEF(Union)
SIMPLE_PATTERN_DEF(UnionX)
SIMPLE_PATTERN_DEF(Difference)
SIMPLE_PATTERN_DEF(Intersection)
SIMPLE_PATTERN_DEF(ReverseDifference)
SIMPLE_PATTERN_DEF(SymmetricDifference)

template<class ClipperFillType, class ConnectType = ConnectTypeOuterFirst>
class OnlyClipPattern :public UnionPattern<IgnoreFillType, ClipperFillType, ConnectType> {
public:
	OnlyClipPattern() = default;
	template<class CFT, class CT>
	OnlyClipPattern(CFT&& cft, CT&& ct) : UnionPattern<
		IgnoreFillType, ClipperFillType, ConnectType
	>(IgnoreFillType{}, std::forward<CFT>(cft), std::forward<CT>(ct)) {
	}
};

template<class SubjectFillType, class ConnectType = ConnectTypeOuterFirst>
class OnlySubjectPattern :public UnionPattern<SubjectFillType, IgnoreFillType, ConnectType> {
public:
	OnlySubjectPattern() = default;
	template<class SFT, class CT>
	OnlySubjectPattern(SFT&& sft, CT&& ct) : UnionPattern<
		SubjectFillType, IgnoreFillType, ConnectType
	>(std::forward<SFT>(sft), IgnoreFillType{}, std::forward<CT>(ct)) {
	}
};

#undef SIMPLE_PATTERN_DEF

TAILOR_NAMESPACE_END