"""Nonblocking SLAM startup with observed lifecycle state as the authority.

Only CONFIGURE/ACTIVATE are sent. A lost transition reply is followed by GET,
never a blind duplicate transition. This code has no motor/navigation API.
"""
class SlamLifecycle:
    def __init__(self, get_client, change_client, get_request, change_request):
        self.get_client=get_client;self.change_client=change_client
        self.get_request=get_request;self.change_request=change_request
        self.pending=None;self.reset()

    def reset(self):
        if self.pending is not None:
            client,future,_,_=self.pending
            client.remove_pending_request(future);future.cancel()
        self.pending=None;self.ready=False;self.state=None;self.reason='slam_waiting_lifecycle_service'
        self.next_poll=0.;self.transition=None;self.timeouts=0

    def tick(self, now):
        if self.pending is not None:
            client,future,kind,deadline=self.pending
            if not future.done() and now<deadline:return
            self.pending=None
            if not future.done():
                client.remove_pending_request(future);future.cancel();self.timeouts+=1
                self.ready=False;self.reason='slam_lifecycle_'+kind+'_reply_timeout'
                # The operation may have succeeded. Re-read its actual state.
                self.transition=None;self.next_poll=now+.2;return
            try:
                result=future.result()
                if kind=='get':
                    self.state=result.current_state.id;self.ready=self.state==3
                    self.reason={1:'slam_unconfigured',2:'slam_inactive',3:'slam_active',4:'slam_finalized'}.get(self.state,'slam_transition_in_progress')
                    self.transition={1:1,2:3}.get(self.state)
                else:
                    self.reason='slam_transition_observe_result' if result.success else 'slam_transition_rejected_recheck'
                    self.transition=None
            except Exception as ex:
                self.ready=False;self.transition=None;self.reason='slam_lifecycle_service_error:'+str(ex)
            self.next_poll=now+(.05 if self.transition is not None else .5)
            return
        if now<self.next_poll:return
        if self.transition is not None:
            if not self.change_client.service_is_ready():
                self.transition=None;self.next_poll=now+.5;return
            request=self.change_request();request.transition.id=self.transition
            self.transition=None;self.ready=False
            self.pending=(self.change_client,self.change_client.call_async(request),'change',now+5.)
        elif self.get_client.service_is_ready():
            self.pending=(self.get_client,self.get_client.call_async(self.get_request()),'get',now+2.)
        else:
            self.ready=False;self.reason='slam_waiting_lifecycle_service';self.next_poll=now+.5
