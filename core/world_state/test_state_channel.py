"""
Unit tests for StateChannel.

These tests validate:
1. Basic state get/set
2. Lazy evaluation
3. Event emission on state changes
4. Subscription management
5. Error handling
"""

import pytest
from core.world_state.state_channel import StateChannel
from core.world_state.event_types import StateChangedEvent


@pytest.fixture
def channel():
    """Create a fresh StateChannel for each test."""
    return StateChannel()


class TestStateChannel:
    """Test StateChannel basic functionality."""
    
    def test_initial_state_is_empty(self, channel):
        """A new StateChannel should have no state."""
        assert len(channel) == 0
        assert channel.get_all_state() == {}
    
    def test_set_state_stores_value(self, channel):
        """Setting state should store the value."""
        channel.set_state("arm_position", [0.1, 0.2, 0.3])
        assert channel.get_state("arm_position") == [0.1, 0.2, 0.3]
    
    def test_get_state_returns_none_for_missing(self, channel):
        """Getting non-existent state should return None."""
        assert channel.get_state("nonexistent") is None
    
    def test_get_state_raises_key_error(self, channel):
        """Getting non-existent state with strict=True should raise KeyError."""
        with pytest.raises(KeyError):
            channel.get_state("nonexistent", strict=True)
    
    def test_state_is_immutable(self, channel):
        """State values should be immutable (frozen)."""
        channel.set_state("config", {"mode": "fast"})
        state = channel.get_state("config")
        with pytest.raises(TypeError):
            state["mode"] = "slow"
    
    def test_state_is_copy_on_write(self, channel):
        """Getting state should return a copy, not the original reference."""
        channel.set_state("data", [1, 2, 3])
        state = channel.get_state("data")
        state.append(4)
        assert channel.get_state("data") == [1, 2, 3]


class TestLazyEvaluation:
    """Test lazy evaluation behavior."""
    
    def test_get_state_triggers_computation(self, channel):
        """Getting state should trigger lazy computation."""
        computed = False
        
        def compute_value():
            nonlocal computed
            computed = True
            return "expensive_value"
        
        channel.register_lazy("expensive_key", compute_value)
        assert computed == False  # Not computed yet
        assert channel.get_state("expensive_key") == "expensive_value"
        assert computed == True  # Now computed
    
    def test_lazy_value_cached(self, channel):
        """Lazy values should be cached after first computation."""
        compute_count = 0
        
        def compute_value():
            nonlocal compute_count
            compute_count += 1
            return f"value_{compute_count}"
        
        channel.register_lazy("cached_key", compute_value)
        assert channel.get_state("cached_key") == "value_1"
        assert channel.get_state("cached_key") == "value_1"  # Cached
        assert compute_count == 1  # Only computed once
    
    def test_lazy_value_invalidation(self, channel):
        """Lazy values should be invalidated when dependencies change."""
        compute_count = 0
        
        def compute_value():
            nonlocal compute_count
            compute_count += 1
            return f"value_{compute_count}"
        
        channel.register_lazy("dynamic_key", compute_value)
        assert channel.get_state("dynamic_key") == "value_1"
        
        channel.invalidate("dynamic_key")
        assert channel.get_state("dynamic_key") == "value_2"
        assert compute_count == 2
    
    def test_set_state_invalidates_lazy(self, channel):
        """Setting state should invalidate dependent lazy values."""
        compute_count = 0
        
        def compute_value():
            nonlocal compute_count
            compute_count += 1
            return f"value_{compute_count}"
        
        channel.register_lazy("dependent_key", compute_value)
        channel.set_state("dependency", [1, 2, 3])
        
        assert channel.get_state("dependent_key") == "value_1"
        channel.set_state("dependency", [4, 5, 6])
        assert channel.get_state("dependent_key") == "value_2"
        assert compute_count == 2


class TestEventEmission:
    """Test event emission on state changes."""
    
    def test_set_state_emits_event(self, channel):
        """Setting state should emit a StateChangedEvent."""
        received_events = []
        channel.subscribe("state_changed", lambda e: received_events.append(e))
        
        channel.set_state("arm_position", [0.1, 0.2, 0.3])
        
        assert len(received_events) == 1
        event = received_events[0]
        assert event.key == "arm_position"
        assert event.old_value is None
        assert event.new_value == [0.1, 0.2, 0.3]
    
    def test_set_state_emits_event_on_update(self, channel):
        """Updating state should emit StateChangedEvent with old/new values."""
        received_events = []
        channel.subscribe("state_changed", lambda e: received_events.append(e))
        
        channel.set_state("arm_position", [0.1, 0.2, 0.3])
        channel.set_state("arm_position", [0.4, 0.5, 0.6])
        
        assert len(received_events) == 2
        event = received_events[1]
        assert event.old_value == [0.1, 0.2, 0.3]
        assert event.new_value == [0.4, 0.5, 0.6]
    
    def test_subscribe_multiple_handlers(self, channel):
        """Multiple subscribers should all receive events."""
        received = []
        channel.subscribe("state_changed", lambda e: received.append("first"))
        channel.subscribe("state_changed", lambda e: received.append("second"))
        
        channel.set_state("key", "value")
        
        assert received == ["first", "second"]
    
    def test_unsubscribe_removes_handler(self, channel):
        """Unsubscribing should prevent future event delivery."""
        received = []
        handler = lambda e: received.append(e)
        
        channel.subscribe("state_changed", handler)
        channel.unsubscribe("state_changed", handler)
        channel.set_state("key", "value")
        
        assert len(received) == 0
    
    def test_event_handler_exception_isolated(self, channel):
        """Exception in one handler should not break others."""
        received = []
        
        def bad_handler(event):
            raise RuntimeError("Handler failed")
        
        def good_handler(event):
            received.append(event)
        
        channel.subscribe("state_changed", bad_handler)
        channel.subscribe("state_changed", good_handler)
        
        channel.set_state("key", "value")
        
        assert len(received) == 1  # Good handler still received event


class TestStateChannelIntegration:
    """Test StateChannel with typed events."""
    
    def test_state_changed_event_type(self, channel):
        """StateChangedEvent should be properly typed."""
        received = []
        channel.subscribe("state_changed", lambda e: received.append(e))
        
        channel.set_state("temperature", 25.5)
        
        assert isinstance(received[0], StateChangedEvent)
        assert received[0].key == "temperature"
        assert received[0].old_value is None
        assert received[0].new_value == 25.5
    
    def test_state_channel_len(self, channel):
        """len() should return number of state entries."""
        assert len(channel) == 0
        channel.set_state("a", 1)
        channel.set_state("b", 2)
        assert len(channel) == 2
    
    def test_state_channel_contains(self, channel):
        """in operator should check if state exists."""
        channel.set_state("key", "value")
        assert "key" in channel
        assert "nonexistent" not in channel


class TestStateChannelEdgeCases:
    """Test edge cases and error handling."""
    
    def test_set_none_value(self, channel):
        """Setting None should be allowed."""
        channel.set_state("key", None)
        assert channel.get_state("key") is None
    
    def test_set_none_key_raises_error(self, channel):
        """Setting None as key should raise error."""
        with pytest.raises(TypeError):
            channel.set_state(None, "value")
    
    def test_empty_key_raises_error(self, channel):
        """Setting empty string as key should raise error."""
        with pytest.raises(ValueError):
            channel.set_state("", "value")
    
    def test_get_state_with_default(self, channel):
        """Getting state with default should return default if missing."""
        assert channel.get_state("missing", default="fallback") == "fallback"
    
    def test_clear_state(self, channel):
        """Clearing state should remove all entries."""
        channel.set_state("a", 1)
        channel.set_state("b", 2)
        channel.clear()
        assert len(channel) == 0
    
    def test_multiple_state_updates_emit_events(self, channel):
        """Multiple updates should emit multiple events."""
        received = []
        channel.subscribe("state_changed", lambda e: received.append(e))
        
        for i in range(10):
            channel.set_state(f"key_{i}", i)
        
        assert len(received) == 10

# Add this at the bottom of test_state_channel.py

if __name__ == "__main__":
    pytest.main([__file__, "-v"])