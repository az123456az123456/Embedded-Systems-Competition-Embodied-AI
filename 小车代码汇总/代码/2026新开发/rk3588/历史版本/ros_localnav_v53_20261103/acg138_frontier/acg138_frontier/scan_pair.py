"""Bounded exact acquisition matching; never pair by receipt proximity."""
from collections import OrderedDict


class ClearingScanCache:
    def __init__(self,capacity=8):
        self.capacity=capacity;self.items=OrderedDict()

    @staticmethod
    def key(m):
        return (m.header.stamp.sec,m.header.stamp.nanosec,m.header.frame_id)

    def clear(self):self.items.clear()

    def put(self,m):
        key=self.key(m);self.items[key]=m;self.items.move_to_end(key)
        while len(self.items)>self.capacity:self.items.popitem(last=False)

    def take(self,m):
        other=self.items.pop(self.key(m),None)
        if other is None:return None
        fields=('angle_min','angle_max','angle_increment','range_min','range_max','scan_time','time_increment')
        if len(other.ranges)!=len(m.ranges) or any(getattr(other,k)!=getattr(m,k) for k in fields):return None
        return other.ranges
