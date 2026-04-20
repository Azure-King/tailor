#pragma once

#include "tailor.h"

namespace detail {

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer, class Container>
class VertexEventQueue;

// VertexEventComparator - needed for VertexEventQueue
template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer, class Container>
class VertexEventComparator {
public:
	using EdgeEvent = ::tailor::EdgeEvent<Edge>;
	using Vertex = edge_analysis_vertex_t<EdgeAnalyzer, Edge>;

	VertexEventComparator(EdgeAnalyzer& ea, std::vector<EdgeEvent>& edge_events) :ea(ea), edgeEvents(edge_events) {
	}

	bool operator()(VertexEventGroup& a, VertexEventGroup& b) {
		return static_cast<int>(RelativePosition(a, b)) < 0;
	}

	auto RelativePosition(VertexEventGroup& a, VertexEventGroup& b) {
		const auto& ap = GetPnt(a);
		const auto& bp = GetPnt(b);
		auto rp = ea.CalcateVertexRelativePosition(ap, bp);
		return rp;
	}

	Vertex GetPnt(VertexEventGroup& p, size_t i = 0) {
		assert(p.edges[i] != npos);
		const auto& edge_event = GetUpdatedEdge(p, i);
		return p.isStart[0] ? ea.Start(edge_event.edge) : ea.End(edge_event.edge);
	}

	/**
	 * @brief 获取更新后的边, 如果更新后的边依旧被废弃, 则说明该边被融合, 有其他的聚合边代替
	 */
	const EdgeEvent& GetUpdatedEdge(VertexEventGroup& p, size_t i = 0) const {
		assert(p.edges[i] != npos);
		const EdgeEvent* event = &edgeEvents[p.edges[i]];
		if (p.isStart[0]) {
			while (event->discarded && event->firstSplit != npos) {
				event = &edgeEvents[event->firstSplit];
				p.edges[i] = event->id;
			}
		} else {
			while (event->discarded && event->secondSplit != npos) {
				event = &edgeEvents[event->secondSplit];
				p.edges[i] = event->id;
			}
		}
		return *event;
	}

	EdgeAnalyzer& ea;
	std::vector<EdgeEvent>& edgeEvents;
};

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer, class Container>
class VertexEventQueue {
public:
	using EdgeEvent = ::tailor::EdgeEvent<Edge>;
	using Vertex = edge_analysis_vertex_t<EdgeAnalyzer, Edge>;

	VertexEventQueue(EdgeAnalyzer& ea, std::vector<EdgeEvent>& edge_events, Container&& container = Container()) :
		ea(ea),
		comparator(ea, edge_events),
		queue(comparator, std::forward<Container>(container)) {
	}

	template<class... Args>
	void Push(Args&&... args) {
		queue.emplace(std::forward<Args>(args)...);
	}

	decltype(auto) Top() const {
		return queue.top();
	}

	decltype(auto) Pop() {
		auto ve = queue.top();
		queue.pop();
		return ve;
	}
	bool Empty() const {
		return queue.empty();
	}

	void PopAllEvents(
		std::vector<VertexEvent>& start_events,
		std::vector<VertexEvent>& end_events) {
		if (Empty()) {
			return;
		}

		VertexEventGroup temp = Pop();
		while (true) {
			Emplace(start_events, end_events, temp);

			if (Empty()) return;
			VertexEventGroup& top = const_cast<VertexEventGroup&>(Top());
			if (::tailor::VertexRelativePositionType::Same !=
				comparator.RelativePosition(temp, top)) return;
			temp = Pop();
		}
	}

public:
	EdgeAnalyzer& ea;
	VertexEventComparator<Edge, EdgeAnalyzer, Container> comparator;
	std::priority_queue<VertexEventGroup, Container, VertexEventComparator<Edge, EdgeAnalyzer, Container>> queue;
private:

	void Emplace(std::vector<VertexEvent>& start_events, std::vector<VertexEvent>& end_events, VertexEventGroup& group) {
		for (size_t i = 0; i < VertexEventGroup::MAX_VERTEX_SIZE; ++i) {
			if (group.edges[i] == npos) return;

			auto& events = group.isStart[i] ? start_events : end_events;
			const auto& edge_event = comparator.GetUpdatedEdge(group, i);
			if (edge_event.discarded) continue;

			events.emplace_back(group.edges[i], group.isStart[i]);
		}
	}
};

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
class EdgeStateSet {
public:
	using EdgeEvent = ::tailor::EdgeEvent<Edge>;
	using Vertex = edge_analysis_vertex_t<EdgeAnalyzer, Edge>;
	using VEQ = VertexEventQueue<Edge, EdgeAnalyzer, std::vector<VertexEventGroup>>;

	EdgeStateSet(VEQ& veq, std::vector<EdgeEvent>& edge_events, EdgeAnalyzer& ea,
		const std::vector<Edge>& clipper, const std::vector<Edge>& subject) :
		veq(veq), edgeEvents(edge_events), ea(ea) {
		size_t edge_size = clipper.size() + subject.size();
		topoVertices.reserve(edge_size * 2.5);
		edgeEvents.reserve(edge_size * 2.5);

		CreateEvents(clipper, subject);
	}

	void PreprocessEvents(std::vector<VertexEvent>& start_events2, std::vector<VertexEvent>& end_events) {
		std::vector<VertexEvent>& start_events = cache;
		start_events.clear();

		// 将起点事件集按顺序排列
		for (const auto& ve : start_events2) {
			InsertVertexEvent(start_events, ve);
		}

		// 1. 在排序前识别所有的聚合边, 并替换对应事件
		// 2. 排序事件, 优先处理相对位置在下方的边
		// 3. 将事件放在活跃边中合适的位置, 求出上下交点(或重合边), 并选取合适的点, 分裂事件边

		// 此处处理一个特殊情况: 起点在某一个活跃边的上
		// 理论上, 此时终点事件集应当为空, 否则先前已经将该边裂解了
		if (!end_events.empty() || start_events.empty()) {
			std::swap(start_events, start_events2);
			return;
		}

		const auto& p = GetPointInVertexEvent(start_events.front());
		for (auto it = events.begin(); it != events.end(); ++it) {
			if (!ea.IsOnEdge(p, GetEdgeEvent(it->e).edge)) continue;

			decltype(auto) result = ea.SplitEdge(GetEdgeEvent(it->e).edge, p);

			// 该分支处理 EdgeRelativePosition 无效的情况, 因为所有起点事件的位置相同,
			// 所以该情况在本函数中应该仅会存在最多一次, 且仅需测试一次

			assert(result.HasPiece(AI));
			assert(result.HasPiece(IB));

			// 裂解 it 指向的边
			auto split_result = SplitEvent(it->e,
				result.GetPiece(AI),
				result.GetPiece(IB)
			);

			// 将 it 指向的起始点事件替换为 AI 的起始点事件
			*it = MakeStartVertexEvent(split_result.aiEvent.id);

			// 终点事件集中加入 AI 的终点事件, 以便后续将其从活跃边中移除
			end_events.emplace_back(MakeEndVertexEvent(split_result.aiEvent.id));

			// 队列中加入 IB 的终点事件
			//veq.Push(MakeEndVertexEvent(split_result.ibEvent.id));

			// 插入事件最后执行, 防止引用失效
			// 将 it 对应的起点事件替换为 IB 的起点事件
			InsertVertexEvent(start_events, MakeStartVertexEvent(split_result.ibEvent.id));

			// 理论上, 活跃边中至多仅可能存在一条边出现此情况
			break;
		}

		std::swap(start_events, start_events2);
	}

	void AcceptSameLocStartEvents(std::vector<VertexEvent>& start_events, TopoVertex& vertex) {
		// 此时起点事件已经排序完毕
		for (const auto& ve : start_events) {
			// 此时插入的边可能后续会被废弃, 但是在结束事件时, 会通过溯源机制, 换回正确的 id
			vertex.startGroup.Insert(ve.e);
			GetEdgeEvent(ve.e).startPntGroup = vertex.id;
		}

		// 二分插入
		auto events_begin = events.begin();
		auto events_end = events.end();
		for (auto& cur : start_events) {
			auto begin = events_begin;
			auto end = events_end;

			auto it = events.begin();
			while (begin < end) {
				// middle
				it = begin + std::distance(begin, end) / 2;

				auto res = ea.CalcateEdgeRelativePosition(
					GetEdgeEvent(it->e).edge,
					GetEdgeEvent(cur.e).edge, // 新边必须放在后面
					OnlyFocusOnRelativePositionWithoutCoincidence{}
				);

				// 上方已经处理了 CI 不存在的情况, 此处应该不会再出现
				//assert(res.HasPiece(CI));

				if (CurveRelativePositionType::Upward == res.RelativePositionType()) {
					begin = it + 1;
				} else {
					end = it;
				}
			}
			it = begin;

			std::optional<CurveRelativePositionResult2<Edge>> lower_intersection;
			if (it != events.begin()) {
				lower_intersection = std::make_optional<CurveRelativePositionResult2<Edge>>(
					ea.CalcateEdgeRelativePosition(
						GetEdgeEvent((it - 1)->e).edge,
						GetEdgeEvent(cur.e).edge,
						OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
					)
				);
			}

			std::optional<CurveRelativePositionResult2<Edge>> upper_intersection;
			if (it != events.end()) {
				upper_intersection = std::make_optional<CurveRelativePositionResult2<Edge>>(
					ea.CalcateEdgeRelativePosition(
						GetEdgeEvent(it->e).edge,
						GetEdgeEvent(cur.e).edge,
						OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
					)
				);
			}

			if (!lower_intersection && !upper_intersection) {
				GroupWindResult{} >> GetEdgeEvent(cur.e);

				// 活跃边为空
				events_begin = events.insert(it, cur);
				events_end = events_begin + 1;

				continue;
			}

			bool use_upper = false;

			if (lower_intersection && !upper_intersection) {
				use_upper = false;
			} else if (!lower_intersection && upper_intersection) {
				use_upper = true;
			} else if (lower_intersection && upper_intersection) {
				bool lower_has_intersection = lower_intersection->HasPiece(IB) || lower_intersection->HasPiece(ID);
				bool upper_has_intersection = upper_intersection->HasPiece(IB) || upper_intersection->HasPiece(ID);

				if (!lower_has_intersection && !upper_has_intersection) {
					CalcGroupWind(events.begin(), it) >> GetEdgeEvent(cur.e);

					auto new_loc = events.emplace(it, cur);
					events_begin = new_loc;
					events_end = events_begin + 1;

					if (events.cbegin() != new_loc) {
						// 如果新加入的活跃边下方有别的活跃边, 则需要记录下方第一条边
						GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
					}
					continue;
				} else if (!lower_has_intersection && upper_has_intersection) {
					use_upper = true;
				} else if (lower_has_intersection && !upper_has_intersection) {
					use_upper = false;
				} else {
					auto I0 = ea.End(lower_intersection->GetPiece(CI));
					auto I1 = ea.End(upper_intersection->GetPiece(CI));

					if (Positive(ea.CalcateVertexRelativePosition(I0, I1))) {
						use_upper = false;
					} else {
						use_upper = true;
					}
				}
			}

			VertexEventGroup group{};
			if (use_upper) {
				if (upper_intersection->HasPiece(IB)) {
					auto spr = SplitEvent(it->e,
						upper_intersection->GetPiece(AI),
						upper_intersection->GetPiece(IB)
					);
					*it = MakeStartVertexEvent(spr.aiEvent.id);

					group.Add(
						spr.aiEvent.id, false,
						spr.ibEvent.id, true
					);
				}

				if (upper_intersection->HasPiece(ID)) {
					auto spr = SplitEvent(cur.e,
						upper_intersection->GetPiece(CI),
						upper_intersection->GetPiece(ID)
					);

					CalcGroupWind(events.begin(), it) >> spr.aiEvent;

					auto new_loc = events.emplace(it, MakeStartVertexEvent(spr.aiEvent.id));
					events_begin = new_loc;
					events_end = events_begin + 1;

					if (events.cbegin() != new_loc) {
						GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
					}

					group.Add(
						spr.aiEvent.id, false,
						spr.ibEvent.id, true
					);
				} else {
					CalcGroupWind(events.begin(), it) >> GetEdgeEvent(cur.e);

					auto new_loc = events.emplace(it, cur);
					events_begin = new_loc;
					events_end = events_begin + 1;

					if (events.cbegin() != new_loc) {
						GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
					}
				}
			} else {
				if (lower_intersection->HasPiece(IB)) {
					auto spr = SplitEvent((it - 1)->e,
						lower_intersection->GetPiece(AI),
						lower_intersection->GetPiece(IB)
					);
					*(it - 1) = MakeStartVertexEvent(spr.aiEvent.id);

					group.Add(
						spr.aiEvent.id, false,
						spr.ibEvent.id, true
					);
				}

				if (lower_intersection->HasPiece(ID)) {
					auto spr = SplitEvent(cur.e,
						lower_intersection->GetPiece(CI),
						lower_intersection->GetPiece(ID)
					);

					CalcGroupWind(events.begin(), it) >> spr.aiEvent;

					auto new_loc = events.emplace(it, MakeStartVertexEvent(spr.aiEvent.id));
					events_begin = new_loc;
					events_end = events_begin + 1;

					if (events.cbegin() != new_loc) {
						GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
					}

					group.Add(
						spr.aiEvent.id, false,
						spr.ibEvent.id, true
					);
				} else {
					CalcGroupWind(events.begin(), it) >> GetEdgeEvent(cur.e);

					auto new_loc = events.emplace(it, cur);
					events_begin = new_loc;
					events_end = events_begin + 1;

					if (events.cbegin() != new_loc) {
						GetEdgeEvent(new_loc->e).firstBottom = (new_loc - 1)->e;
					}
				}
			}
			if (group.edges[0] != npos) {
				veq.Push(group);
			}
		}
	}

	void AcceptSameLocEndEvents(std::vector<VertexEvent>& end_events, TopoVertex& vertex) {
		for (auto begin = events.begin(); begin != events.end(); ++begin) {
			if (end_events.empty()) break;

			auto end = begin; // 需要移除的区间末尾
			auto event_end = events.end();
			for (; !end_events.empty() && end != events.end(); ++end) {
				auto remove_it = std::remove_if(end_events.begin(), end_events.end(),
					[eid = end->e](const VertexEvent& ve) {return ve.e == eid; }
				);
				if (end_events.end() == remove_it) break;
				end_events.erase(remove_it, end_events.end());
			}

			// 没有需要要删除的事件
			if (begin == end) continue;

			// 结束连续的事件
			std::for_each(begin, end, [this, &vertex](const VertexEvent& ve) {
				EdgeEvent& ee = GetEdgeEvent(ve.e);
				ee.endPntGroup = vertex.id;
				EndEvent(ee); // 结束被移除的事件
				});

			// 删除 [begin, end)
			begin = events.erase(begin, end);

			if (events.end() == begin /* 上方无边 */) break;
			if (events.begin() == begin /* 下方无边 */) continue;

			//VertexRelativePositionType vrp = ea.CalcateVertexRelativePosition((begin - 1)->v, begin->v);
			const ::tailor::VertexRelativePositionType vrp = ea.CalcateVertexRelativePosition(
				GetPointInVertexEvent(*(begin - 1)), GetPointInVertexEvent(*begin)
			);

			const bool reversed = Negative(vrp);

			// 由于比较的两个事件可能不满足 A <= C, 因此需要判断
			const auto ab_it = reversed ? begin : (begin - 1);
			const auto cd_it = reversed ? (begin - 1) : begin;

			// 移除事件后, 将上下的边事件进行求交处理
			auto erp = ea.CalcateEdgeRelativePosition(
				GetEdgeEvent(ab_it->e).edge,
				GetEdgeEvent(cd_it->e).edge,
				OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
			);

			//const auto& debug_edge0 = GetEdgeEvent(ab_it->e);
			//const auto& debug_edge1 = GetEdgeEvent(cd_it->e);
			assert(erp.HasPiece(CI));

			VertexEventGroup group{};
			if (erp.HasPiece(IB)) {
				auto spr = SplitEvent(ab_it->e, erp.GetPiece(AI), erp.GetPiece(IB));

				// 将 ab_it 指向的起始点事件替换为 AI 的起始点事件
				*ab_it = MakeStartVertexEvent(spr.aiEvent.id);

				group.Add(
					spr.aiEvent.id, false,
					spr.ibEvent.id, true
				);

				// 重新加入事件: AI 的终点事件、IB 的起点事件、IB 的终点事件
				//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
				//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
				//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
			}

			if (erp.HasPiece(ID)) {
				auto spr = SplitEvent(cd_it->e, erp.GetPiece(CI), erp.GetPiece(ID));

				// 将 cd_it 指向的起始点事件替换为 CI 的起始点事件
				*cd_it = MakeStartVertexEvent(spr.aiEvent.id);

				group.Add(
					spr.aiEvent.id, false,
					spr.ibEvent.id, true
				);

				// 重新加入事件: CI 的终点事件、ID 的起点事件、ID 的终点事件
				//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
				//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
				//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
			}
			if (group.edges[0] != npos) {
				veq.Push(group);
			}
		}
	}

	inline const EdgeEvent& GetEdgeEvent(Handle handle) const {
		return edgeEvents[handle];
	}
	inline EdgeEvent& GetEdgeEvent(Handle handle) {
		return edgeEvents[handle];
	}
	inline const TopoVertex& GetVertexEvent(Handle handle) const {
		return topoVertices[handle];
	}
	inline TopoVertex& GetVertexEvent(Handle handle) {
		return topoVertices[handle];
	}

	VertexEvent MakeStartVertexEvent(Handle e) {
		return VertexEvent{ e, true };
	}

	VertexEvent MakeEndVertexEvent(Handle e) {
		return VertexEvent{ e, false };
	}

	template<class E>
	EdgeEvent MakeEdgeEvent(E&& e, Handle id, bool is_clipper) {
		EdgeEvent ee{ std::forward<E>(e) };
		ee.id = id;
		ee.isClipper = is_clipper;
		return ee;
	}

	template<class E>
	inline EdgeEvent& RegisterEdge(E&& edge) {
		Handle edge_handle = static_cast<Handle>(edgeEvents.size());
		return edgeEvents.emplace_back(std::forward<E>(edge), edge_handle);
	}

	template<class... E>
	auto RegisterEdge(E&&... edge) {
		size_t start = edgeEvents.size();
		(RegisterEdge(std::forward<E>(edge)), ...);
		return GetEdgeEvents(start, std::make_index_sequence<sizeof...(E)>());
	}

	auto& RegisterVertex() {
		Handle id = static_cast<Handle>(topoVertices.size());
		auto& res = topoVertices.emplace_back();
		res.id = id;
		return res;
	}

	template<size_t... Index>
	auto GetEdgeEvents(size_t start, std::index_sequence<Index...>) {
		return std::tie(edgeEvents[start + Index]...);
	}

	decltype(auto) GetPointInVertexEvent(const VertexEvent& ve) {
		assert(!GetEdgeEvent(ve.e).discarded);
		return ve.start ? ea.Start(GetEdgeEvent(ve.e).edge) : ea.End(GetEdgeEvent(ve.e).edge);
	}

	struct CopyGroup {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.isClipper = base.isClipper;
		}
	};

	struct CopyFirstBottomEdge {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.firstBottom = base.firstBottom;
		}
	};

	struct TrackFirstSegmentedEdge {
		inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
			base.firstSplit = derived.id;
		}
	};

	struct MergeFirstSegmentedEdge {
		inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
			base.firstMerge = derived.id;
		}
	};
	struct TrackSecondSegmentedEdge {
		inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
			base.secondSplit = derived.id;
		}
	};

	struct CopyWinds {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.clipperWind = base.clipperWind;
			derived.subjectWind = base.subjectWind;
		}
	};

	struct CopyPolarity {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.reversed = base.reversed;
		}
	};

	struct InvertPolarity {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.reversed = !base.reversed;
		}
	};

	struct CopyMonotonicity {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.monotonicity = base.monotonicity;
		}
	};

	struct InvertMonotonicity {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.monotonicity = static_cast<::tailor::VertexRelativePositionType>(
				-static_cast<int>(base.monotonicity)
				);
		}
	};

	struct CopySource {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.source = base.id;
		}
	};

	struct CopyStartingVertex {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.startPntGroup = base.startPntGroup;
		}
	};

	struct CopyEndVertex {
		inline void operator()(EdgeEvent& derived, const EdgeEvent& base) const {
			derived.endPntGroup = base.endPntGroup;
		}
	};

	struct DiscardBase {
		inline void operator()(const EdgeEvent& derived, EdgeEvent& base) const {
			base.discarded = true;
		}
	};

	struct CopyAggregatedEdges {
		inline void operator()(EdgeEvent& derived, EdgeEvent& base) const {
			if (base.aggregatedEdges) {
				derived.aggregatedEdges = std::make_unique<AggregatedEdgeEvent>(*base.aggregatedEdges);
			}
		}
	};

	struct MergeAggregatedEdges {
		void operator()(EdgeEvent& derived, EdgeEvent& base) const {
			if (!derived.IsAggregatedEdge()) {
				derived.aggregatedEdges = std::make_unique<AggregatedEdgeEvent>();
				derived.aggregatedEdges->sourceEdges.reserve(2);
				derived.aggregatedEdges->sourceEdges.emplace_back(derived.id);
			}

			auto& das = derived.aggregatedEdges->sourceEdges;
			if (base.IsAggregatedEdge()) {
				auto& bas = base.aggregatedEdges->sourceEdges;
				das.insert(das.end(), bas.begin(), bas.end());
			} else {
				das.emplace_back(base.id);
			}
		}
	};

	template<class... Inherited>
	inline void Inherit(EdgeEvent& derived, EdgeEvent& base) {
		(Inherited{}(derived, base), ...);
	}

	struct HandleCoincidentEdgesResult {
		Handle coincidentEdges = npos;
		Handle ib = npos;
		Handle id = npos;
	};

	template<class EdgeRelativePositionResult>
	HandleCoincidentEdgesResult HandleCoincidentEdges(Handle ab_handle, Handle cd_handle, EdgeRelativePositionResult&& result) {
		assert(result.HasPiece(CI));
		assert(CurveRelativePositionType::Coincident == result.RelativePositionType());

		if (result.HasPiece(IB) && result.HasPiece(ID)) {
			auto [ai_event, ib_event, id_event] = RegisterEdge(
				result.GetPiece(AI), result.GetPiece(IB), result.GetPiece(ID)
			);

			auto& ab_event = GetEdgeEvent(ab_handle);
			auto& cd_event = GetEdgeEvent(cd_handle);

			Inherit<
				DiscardBase, CopySource, CopyGroup,
				CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
				TrackSecondSegmentedEdge
			>(ib_event, ab_event);

			Inherit<
				DiscardBase, CopySource, CopyGroup,
				CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
				TrackSecondSegmentedEdge
			>(id_event, cd_event);

			Inherit<
				DiscardBase, CopySource, CopyGroup, CopyPolarity,
				CopyMonotonicity, CopyAggregatedEdges,
				TrackFirstSegmentedEdge
			>(ai_event, ab_event);

			Inherit<MergeAggregatedEdges, MergeFirstSegmentedEdge>(ai_event, cd_event);

			return HandleCoincidentEdgesResult{ ai_event.id, ib_event.id, id_event.id };
		} else if (!result.HasPiece(IB) && result.HasPiece(ID)) {
			auto& id_event = RegisterEdge(result.GetPiece(ID));
			auto& ab_event = GetEdgeEvent(ab_handle);
			auto& cd_event = GetEdgeEvent(cd_handle);

			Inherit<
				DiscardBase, CopySource, CopyGroup,
				CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
				TrackSecondSegmentedEdge
			>(id_event, cd_event);

			Inherit<MergeAggregatedEdges, MergeFirstSegmentedEdge>(ab_event, cd_event);

			return HandleCoincidentEdgesResult{ ab_event.id, npos, id_event.id };
		} else if (result.HasPiece(IB) && !result.HasPiece(ID)) {
			auto& ib_event = RegisterEdge(result.GetPiece(IB));
			auto& ab_event = GetEdgeEvent(ab_handle);
			auto& cd_event = GetEdgeEvent(cd_handle);

			Inherit<
				DiscardBase, CopySource, CopyGroup,
				CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
				TrackSecondSegmentedEdge
			>(ib_event, ab_event);

			Inherit<MergeAggregatedEdges, MergeFirstSegmentedEdge>(cd_event, ab_event);

			return HandleCoincidentEdgesResult{ cd_event.id, ib_event.id, npos };
		} else {
			auto& ab_event = GetEdgeEvent(ab_handle);
			auto& cd_event = GetEdgeEvent(cd_handle);

			Inherit<
				DiscardBase, MergeAggregatedEdges,
				MergeFirstSegmentedEdge
			>(ab_event, cd_event);

			return HandleCoincidentEdgesResult{ ab_event.id, npos, npos };
		}
	}

	struct GroupWindResult {
		Int clipperWind = 0;
		Int subjectWind = 0;

		void operator>>(EdgeEvent& ee) const {
			ee.clipperWind = clipperWind;
			ee.subjectWind = subjectWind;
		}
	};

	template<class EventsIterator>
	GroupWindResult CalcGroupWind(EventsIterator begin, EventsIterator end) const {
		GroupWindResult winds{};
		if (begin == end) return winds;
		const auto& ee = GetEdgeEvent((end - 1)->e);
		winds.clipperWind = ee.clipperWind;
		winds.subjectWind = ee.subjectWind;
		if (ee.aggregatedEdges) TAILOR_UNLIKELY{
			for (auto ae : ee.aggregatedEdges->sourceEdges) {
				auto& aee = GetEdgeEvent(ae);
				auto& wind = aee.isClipper ? winds.clipperWind : winds.subjectWind;
				wind += aee.reversed ? -1 : +1;
			}
		} else {
			auto& wind = ee.isClipper ? winds.clipperWind : winds.subjectWind;
			wind += ee.reversed ? -1 : +1;
		}
		return winds;
	}

	void CreateEvents(const std::vector<Edge>& clipper, const std::vector<Edge>& subject) {
		for (const auto& e : clipper) {
			RegisterEdge(e).isClipper = true;
		}
		for (const auto& e : subject) {
			RegisterEdge(e).isClipper = false;
		}

		for (size_t i = 0, n = edgeEvents.size(); i < n; ++i) {
			auto& old_event = GetEdgeEvent(i);
			if (old_event.discarded) continue;

			auto& edge = edgeEvents[i].edge;
			old_event.monotonicity = ea.CalcateVertexRelativePosition(ea.Start(edge), ea.End(edge));

			if (!Negative(old_event.monotonicity)) continue;

			auto& new_event = RegisterEdge(ea.Reverse(edge));

			Inherit<
				DiscardBase, CopySource, CopyGroup,
				InvertPolarity, InvertMonotonicity
			>(new_event, GetEdgeEvent(i));
		}
	}

	struct SplitEventResult {
		EdgeEvent& aiEvent;
		EdgeEvent& ibEvent;
	};

	template<class E0, class E1>
	auto SplitEvent(Handle ab_handle, E0&& ai, E1&& ib) {
		auto [ai_event, ib_event] = RegisterEdge(
			std::forward<E0>(ai), std::forward<E1>(ib)
		);
		EdgeEvent& ab_event = GetEdgeEvent(ab_handle);

		Inherit<
			DiscardBase, CopySource, CopyGroup, CopyWinds,
			CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
			CopyStartingVertex, CopyEndVertex, CopyFirstBottomEdge,
			TrackFirstSegmentedEdge
		>(ai_event, ab_event);
		Inherit<
			DiscardBase, CopySource, CopyGroup, CopyWinds,
			CopyPolarity, CopyMonotonicity, CopyAggregatedEdges,
			TrackSecondSegmentedEdge
		>(ib_event, ab_event);
		return SplitEventResult{ ai_event, ib_event };
	}

	void EndEvent(EdgeEvent& e) {
		e.end = true;

		TopoVertex& end_vertex = GetVertexEvent(e.endPntGroup);
		end_vertex.endGroup.Insert(e.id);

		TopoVertex& start_vertex = GetVertexEvent(e.startPntGroup);
		bool success = start_vertex.startGroup.ReplaceEvent(e.id, this->edgeEvents);
		assert(success);
	}

	void InsertVertexEvent(std::vector<VertexEvent>& start_events, const VertexEvent& new_event) {
		if (GetEdgeEvent(new_event.e).discarded) return;

		if (start_events.empty()) {
			start_events.emplace_back(new_event);
			return;
		}

		auto begin = start_events.begin();
		auto end = start_events.end();

		while (begin < end) {
			auto it = begin + std::distance(begin, end) / 2;

			auto rp = ea.CalcateEdgeRelativePosition(
				GetEdgeEvent(it->e).edge, GetEdgeEvent(new_event.e).edge,
				OnlyFocusOnRelativePositionAndCoincidenceDetails{}
			);

			if (CurveRelativePositionType::Coincident == rp.RelativePositionType()) {
				auto info = HandleCoincidentEdges(it->e, new_event.e, rp);

				auto& old_edge = GetEdgeEvent(it->e);

				VertexEventGroup group{};

				if (old_edge.discarded) {
					*it = MakeStartVertexEvent(info.coincidentEdges);
				}

				if (npos != info.ib) {
					group.Add(info.ib, true);
				}

				if (npos != info.id) {
					group.Add(info.id, true);
				}

				if (group.edges[0] != npos) {
					veq.Push(group);
				}
				return;
			}

			if (rp.RelativePositionType() == CurveRelativePositionType::Upward) {
				begin = it + 1;
			} else {
				end = it;
			}
		}

		start_events.emplace(begin, new_event);
	}

	size_t CalcVaildEdgeSize(const std::vector<VertexEvent>& es) const {
		return std::accumulate(es.begin(), es.end(), 0, [this](size_t count, const VertexEvent& e) {
			const auto& edge = GetEdgeEvent(e.e);
			return count + (edge.aggregatedEdges ?
				edge.aggregatedEdges->sourceEdges.size() : 1);
			});
	}

public:
	std::vector<VertexEvent> events;
	EdgeAnalyzer& ea;
	VEQ& veq;
	std::vector<VertexEvent> cache;

	std::vector<EdgeEvent>& edgeEvents;
	std::vector<TopoVertex> topoVertices;
};

} // namespace detail

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
inline auto Tailor<Edge, EdgeAnalyzer>::Execute() -> PatternDrafting {
	using VEQ = detail::VertexEventQueue<Edge, EdgeAnalyzer, std::vector<VertexEventGroup>>;
	using EdgeEvent = ::tailor::EdgeEvent<Edge>;

	std::vector<VertexEventGroup> container;
	std::vector<EdgeEvent> edge_events;

	//container.reserve(20000);
	container.reserve(static_cast<size_t>((clipper.size() + subject.size()) * 2.5));
	VEQ queue(ea, edge_events, std::move(container));
	detail::EdgeStateSet<Edge, EdgeAnalyzer> set(queue, edge_events, ea, clipper, subject);

	// 顶点事件入队

	for (size_t i = 0, n = set.edgeEvents.size(); i < n; ++i) {
		auto& edge = set.edgeEvents[i];
		if (edge.discarded) continue;
		queue.Push(i, true);
		queue.Push(i, false);
		//queue.Push(ea.Start(edge.edge), i, true);
		//queue.Push(ea.End(edge.edge), i, false);
	}

	std::vector<VertexEvent> start_events;
	std::vector<VertexEvent> end_events;

	//size_t times = 0;
	// 顶点事件出队
	while (!queue.Empty()) {
		//times++;

		//if (times == 130) {
		//	int ccc = 0;
		//}

		// 取出所有的相同位置的顶点事件
		queue.PopAllEvents(start_events, end_events);
		// 现在应该可以禁用 RemoveDiscardedEvent 了
		//set.RemoveDiscardedEvent(start_events);
		//set.RemoveDiscardedEvent(end_events);

		// 事件处理的有效边总数必须为偶数
		assert((
			set.CalcVaildEdgeSize(start_events) +
			set.CalcVaildEdgeSize(end_events))
			% 2 == 0);

		//for (auto& se : start_events)
		//{
		//	const auto& edge = set.GetEdgeEvent(se.e);
		//	this->DebugFunc3(edge, se.start);
		//}

		//for (auto& ee : end_events)
		//{
		//	const auto& edge = set.GetEdgeEvent(ee.e);
		//	this->DebugFunc3(edge, ee.start);
		//}

		auto& vertex = set.RegisterVertex();

		set.PreprocessEvents(start_events, end_events);

		set.AcceptSameLocEndEvents(end_events, vertex);

		set.AcceptSameLocStartEvents(start_events, vertex);

		end_events.clear();
		start_events.clear();
	}
	return PatternDrafting{ std::move(edge_events),std::move(set.topoVertices) };
}
