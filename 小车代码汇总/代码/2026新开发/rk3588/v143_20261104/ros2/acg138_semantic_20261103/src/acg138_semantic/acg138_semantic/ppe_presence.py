"""Label-presence rule ported from alerts/src/ppe_core.h::Engine::observe.

Use the voice listener's 0.35 score threshold, fresh complete YOLO results,
three distinct observations and disappearance reset. Search requires both
labels in each result and measures >500 ms of source observations. No display
overlay, box association or playback outcome participates in stopping.
"""
from .vision import number


class VestPresence:
    labels = ('Person','NO-Safety Vest')
    confidence = .35
    confirm_ms = 500
    max_age_ms = 1000

    def __init__(self, started):
        self.started = started
        self.epoch = None
        self.last_stamp = 0
        self.last_frame = 0
        self.last_valid = 0.
        self.first = None
        self.samples = 0

    def reset_presence(self):
        self.first = None
        self.samples = 0

    def observe(self, meta, now):
        if not isinstance(meta,dict):
            self.reset_presence();return []
        stamp=meta.get('detection_timestamp_ms');fid=meta.get('detections_frame_id')
        epoch=meta.get('source_epoch');detections=meta.get('detections')
        if (meta.get('demo') is not False or meta.get('control_eligible') is not True
                or meta.get('detections_pair_status')!='PAIR_COMPLETE'
                or type(stamp) is not int or stamp<=0 or type(fid) is not int or fid<=0
                or type(epoch) is not int or epoch<0
                or not isinstance(detections,list) or len(detections)>256
                or stamp/1000.<self.started or not 0<=now-stamp/1000.<=self.max_age_ms/1000.):
            self.reset_presence();return []
        if epoch!=self.epoch:
            self.reset_presence();self.last_stamp=self.last_frame=0;self.epoch=epoch
        # Preview copies of the same detector result cannot extend its dwell.
        if stamp<=self.last_stamp or fid<=self.last_frame:
            return []
        if stamp-self.last_stamp>self.max_age_ms:
            self.reset_presence()
        self.last_stamp,self.last_frame,self.last_valid=stamp,fid,stamp/1000.
        found=set()
        for d in detections:
            if not isinstance(d,dict):continue
            label,score=d.get('label'),d.get('score')
            if label in self.labels and number(score) and self.confidence<=score<=1:
                found.add(label)
        if not all(label in found for label in self.labels):
            self.reset_presence();return []
        if self.first is None:self.first=stamp
        self.samples+=1
        if self.samples>=3 and stamp-self.first>self.confirm_ms:
            return [dict(labels=list(self.labels),frame=fid,stamp=stamp/1000.,
                         first_stamp=self.first/1000.,source_epoch=epoch,
                         confirmation='voice_label_presence',duration_ms=stamp-self.first)]
        return []
