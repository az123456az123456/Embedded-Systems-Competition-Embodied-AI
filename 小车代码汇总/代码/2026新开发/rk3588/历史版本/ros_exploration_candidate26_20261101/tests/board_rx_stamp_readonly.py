"""Observe actual kernel SNS1 timestamps; never transmit or alter clock."""
import binascii,json,socket,struct,time
s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x88b8));s.bind(('eth0',0));s.settimeout(.5)
s.setsockopt(socket.SOL_SOCKET,getattr(socket,'SO_TIMESTAMPNS',35),1)
rows=[];end=time.monotonic()+5
while time.monotonic()<end and len(rows)<150:
    try:data,anc,flags,address=s.recvmsg(4096,256)
    except TimeoutError:continue
    p=data[14:80]
    if len(p)!=66 or p[:5]!=b'SNS1\x01' or binascii.crc_hqx(p[:64],0xffff)!=int.from_bytes(p[64:66],'big'):continue
    timestamp=None
    for level,kind,value in anc:
        if level==socket.SOL_SOCKET and kind==getattr(socket,'SO_TIMESTAMPNS',35):
            sec,ns=struct.unpack('qq',value[:16]);timestamp=sec*1000000000+ns
    rows.append({'seq':int.from_bytes(p[8:12],'big'),'kernel_ns':timestamp,'receipt_ns':time.time_ns(),'mono_ns':time.monotonic_ns(),'illegal':p[62]})
s.close()
bad=[]
for i in range(1,len(rows)):
    a,b=rows[i-1],rows[i]
    if b['kernel_ns'] is None or a['kernel_ns'] is None or b['kernel_ns']<=a['kernel_ns']:bad.append({'before':a,'after':b})
print(json.dumps({'case':'real_kernel_RX_timestamps','read_only':True,'samples':len(rows),'non_increasing':bad,'first':rows[:3],'last':rows[-3:]}),flush=True)
