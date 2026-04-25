"""
State Channel - Generic publish/subscribe for application events.
Used for notifications that aren't transform-related.
Examples: joint slider moved, camera started, UI updated.
"""

from typing import Dict, List, Callable, Any, Optional
from collections import defaultdict
from enum import Enum
from dataclasses import dataclass
from .event_types import EventType
import time

@dataclass
class Event:
    """An event with metadata."""
    type: EventType
    data: Any
    source: str
    timestamp: float
    description: str = ""

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


    def __str__(self):
        return f"[{self.timestamp:.2f}] {self.source}: {self.type.value} - {self.description}"


class StateChannel:
    """
    Generic publish/subscribe system for application events.

    Usage:
        channel = StateChannel()

        # Subscribe to events
        def on_joint_update(event):
            print(f"Joint moved: {event.data}")

        channel.subscribe(EventType.JOINT_UPDATE, on_joint_update)

        # Publish events
        channel.publish(
            EventType.JOINT_UPDATE,
            data={"joint": "shoulder", "position": 1.5},
            source="joint_control_panel"
        )
    """

    def __init__(self, enable_history: bool = False, max_history: int = 100):
        """
        Initialize state channel.

        Args:
            enable_history: Whether to store event history
            max_history: Maximum number of events to keep in history
        """
        self._subscribers: Dict[EventType, List[Callable[[Event], None]]] = defaultdict(list)
        self._wildcard_subscribers: List[Callable[[Event], None]] = []
        self._history: List[Event] = []
        self._enable_history = enable_history
        self._max_history = max_history

    def subscribe(self, event_type: EventType, callback: Callable[[Event], None]) -> None:
        """
        Subscribe to a specific event type.

        Args:
            event_type: Type of event to subscribe to
            callback: Function that takes an Event object
        """
        if callback not in self._subscribers[event_type]:
            self._subscribers[event_type].append(callback)
            print(f"StateChannel: Subscribed to {event_type.value}")

    def subscribe_all(self, callback: Callable[[Event], None]) -> None:
        """
        Subscribe to ALL events.

        Args:
            callback: Function that takes an Event object
        """
        if callback not in self._wildcard_subscribers:
            self._wildcard_subscribers.append(callback)
            print("StateChannel: Subscribed to ALL events")

    def unsubscribe(self, event_type: EventType, callback: Callable[[Event], None]) -> None:
        """Unsubscribe from a specific event type."""
        if callback in self._subscribers[event_type]:
            self._subscribers[event_type].remove(callback)

    def unsubscribe_all(self, callback: Callable[[Event], None]) -> None:
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
            event_type: Type of event
            data: Event data (any Python object)
            source: Name of the component publishing the event
            description: Human-readable description
        """
        event = Event(
            type=event_type,
            data=data,
            source=source,
            timestamp=time.time(),
            description=description or f"{source} published {event_type.value}"
        )

        # Store in history if enabled
        if self._enable_history:
            self._history.append(event)
            # Trim history if needed
            if len(self._history) > self._max_history:
                self._history.pop(0)

        # Notify wildcard subscribers (subscribed to all)
        for callback in self._wildcard_subscribers:
            try:
                callback(event)
            except Exception as e:
                print(f"Error in wildcard subscriber: {e}")

        # Notify type-specific subscribers
        for callback in self._subscribers[event_type]:
            try:
                callback(event)
            except Exception as e:
                print(f"Error in subscriber for {event_type.value}: {e}")

        # Print debug for important events
        if event_type in [EventType.ERROR_OCCURRED, EventType.CAMERA_STARTED, EventType.CAMERA_STOPPED]:
            print(f"📢 {event}")

    def get_history(self, event_type: Optional[EventType] = None, limit: int = 10) -> List[Event]:
        """
        Get recent event history.

        Args:
            event_type: Filter by event type (None for all)
            limit: Maximum number of events to return

        Returns:
            List of recent events
        """
        if not self._enable_history:
            return []

        if event_type is None:
            return self._history[-limit:]

        filtered = [e for e in self._history if e.type == event_type]
        return filtered[-limit:]

    def clear_history(self) -> None:
        """Clear event history."""
        self._history.clear()

    def get_subscriber_count(self, event_type: Optional[EventType] = None) -> int:
        """Get number of subscribers for an event type."""
        if event_type is None:
            return len(self._wildcard_subscribers)
        return len(self._subscribers[event_type])


# Convenience decorator for subscribing
def on_event(event_type: EventType):
    """Decorator for subscribing to events."""
    def decorator(func):
        func._event_subscription = event_type
        return func
    return decorator
