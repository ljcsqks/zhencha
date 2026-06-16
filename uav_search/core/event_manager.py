"""
事件管理器模块

实现了优先级事件队列，用于管理系统的各种事件。
支持防抖机制，避免相同事件的频繁触发。

主要功能：
- 事件优先级管理（高优先级事件优先处理）
- 事件防抖（短时间内相同事件只处理一次）
- 时间窗口查询（获取指定时间前的事件）
"""
from __future__ import annotations

import heapq

from uav_search.core.data_types import Event


class EventManager:
    """事件管理器类

    基于优先队列的事件管理系统，支持事件优先级和防抖机制。

    属性：
        debounce_s: 防抖时间窗口（秒）
        _queue: 优先队列，存储待处理事件
        _last_event_time: 记录每种类型事件的最后触发时间

    设计思路：
        - 使用堆实现优先队列，支持O(log n)的插入和取出
        - 防抖机制避免短时间内相同事件重复触发
        - 按时间戳顺序处理事件，支持时间窗口查询

    使用示例：
        >>> manager = EventManager(debounce_s=0.2)
        >>> event = Event(id="1", type=EventType.LOW_BATTERY, ...)
        >>> manager.emit(event)  # 添加事件
        >>> events = manager.poll_events(now=10.0)  # 获取当前时间前的事件
    """

    def __init__(self, debounce_s: float = 0.2) -> None:
        """初始化事件管理器

        参数：
            debounce_s: 防抖时间窗口（秒），默认0.2秒
                       相同类型和来源的事件在此时间窗口内只会被保留一次
        """
        self.debounce_s = max(0.0, float(debounce_s))
        self._queue: list[Event] = []
        self._last_event_time: dict[tuple[str, str | None], float] = {}

    def emit(self, event: Event) -> bool:
        """发射事件到队列

        将事件添加到优先队列，支持防抖机制。

        参数：
            event: 待添加的事件对象

        返回：
            bool: 是否成功添加（被防抖过滤返回False）

        防抖规则：
            - 事件键 = (事件类型, 触发无人机ID)
            - 如果相同键的事件在debounce_s内已发射，则忽略新事件
            - 避免短时间内重复处理相同事件
        """
        key = (event.type.value, event.source_uav_id)
        last_time = self._last_event_time.get(key)
        if last_time is not None and event.timestamp - last_time < self.debounce_s:
            return False
        self._last_event_time[key] = event.timestamp
        heapq.heappush(self._queue, event)
        return True

    def poll_events(self, current_time: float | None = None) -> list[Event]:
        """查询待处理事件

        从队列中取出所有待处理事件，支持时间窗口过滤。

        参数：
            current_time: 当前时间戳，None表示不限制时间
                         只返回timestamp <= current_time的事件

        返回：
            list[Event]: 待处理事件列表，按优先级和时间戳排序

        注意：
            - 取出的事件会从队列中移除
            - 如果指定current_time，超过该时间的事件会保留在队列中
        """
        events: list[Event] = []
        while self._queue:
            event = heapq.heappop(self._queue)
            if current_time is not None and event.timestamp > current_time:
                # 事件时间超过当前时间，放回队列并停止处理
                heapq.heappush(self._queue, event)
                break
            events.append(event)
        return events

    def has_events(self) -> bool:
        """检查是否有待处理事件

        返回：
            bool: 队列中是否有事件
        """
        return bool(self._queue)
