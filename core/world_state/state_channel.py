"""
State Channel - Generic publish/subscribe for application events.

Principle: Time = StateChannel. All events flow through here.
Principle: Event-Driven, No Polling.
"""

import time
import logging
from typing import Dict, List, Callable, Any, Optional
from collections import defaultdict

from .event_types import EventType

logger = logging.getLogger(__name__)


class Event:
    """An event with metadata."""

    def __init__(self, type, data, source, timestamp=None, description=""):
        self.type = type
        self.data = data
        self.source = source
        self.timestamp = timestamp or time.time()
        self.description = description or f"{source} published {type.value}"

    def __str__(self):
        return (f"[{self.timestamp:.2f}] {self.source}: "
                f"{self.type.value} - {self.description}")


class StateChannel:
    """
    Generic publish/subscribe system for application events.

    Not thread-safe. Designed for single-threaded use per Hatch architecture.

    Usage:
        channel = StateChannel()

        def on_robot_state(event):
            print(f"Robot state: {event.data}")

        channel.subscribe(EventType.ROBOT_STATE, on_robot_state)
        channel.publish(EventType.ROBOT_STATE, data={...}, source="robot")
    """

    def __init__(self, enable_history: bool = False, max_history: int = 100):
        self._subscribers: Dict[EventType, List[Callable]] = defaultdict(list)
        self._wildcard_subscribers: List[Callable] = []
        self._history: List[Event] = []
        self._enable_history = enable_history
        self._max_history = max_history

    def subscribe(self, event_type: EventType, callback: Callable) -> None:
        """Subscribe to a specific event type."""
        if callback not in self._subscribers[event_type]:
            self._subscribers[event_type].append(callback)

    def subscribe_all(self, callback: Callable) -> None:
        """Subscribe to ALL events."""
        if callback not in self._wildcard_subscribers:
            self._wildcard_subscribers.append(callback)

    def unsubscribe(self, event_type: EventType, callback: Callable) -> None:
        """Unsubscribe from a specific event type."""
        if callback in self._subscribers[event_type]:
            self._subscribers[event_type].remove(callback)

    def unsubscribe_all(self, callback: Callable) -> None:
        """Unsubscribe from all events."""
        if callback in self._wildcard_subscribers:
            self._wildcard_subscribers.remove(callback)

    def publish(self,
                event_type: EventType,
                data: Any = None,
                source: str = "unknown",
                description: str = "") -> None:
        """
        Publish an event to all subscribers.

        Args:
            event_type: Type of event.
            data: Event payload.
            source: Name of the publishing component.
            description: Human-readable description.
        """
        event = Event(
            type=event_type,
            data=data,
            source=source,
            timestamp=time.time(),
            description=description
        )

        if self._enable_history:
            self._history.append(event)
            if len(self._history) > self._max_history:
                self._history.pop(0)

        # Notify wildcard subscribers
        for callback in self._wildcard_subscribers:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Error in wildcard subscriber: {e}")

        # Notify type-specific subscribers
        for callback in self._subscribers[event_type]:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Error in subscriber for {event_type.value}: {e}")

    def get_history(self,
                    event_type: Optional[EventType] = None,
                    limit: int = 10) -> List[Event]:
        """Get recent event history."""
        if not self._enable_history:
            return []

        if event_type is None:
            return self._history[-limit:]

        filtered = [e for e in self._history if e.type == event_type]
        return filtered[-limit:]

    def clear_history(self) -> None:
        """Clear event history."""
        self._history.clear()

    def get_subscriber_count(self,
                             event_type: Optional[EventType] = None) -> int:
        """Get number of subscribers."""
        if event_type is None:
            return len(self._wildcard_subscribers)
        return len(self._subscribers[event_type])