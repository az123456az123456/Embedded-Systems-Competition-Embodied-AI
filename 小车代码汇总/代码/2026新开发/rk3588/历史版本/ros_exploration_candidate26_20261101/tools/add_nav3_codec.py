"""One-shot approved extension generator; generated C++ remains the source."""
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'common/ros_bridge_codec.cc';s=p.read_text()
if 'case ARB1_KIND_MISSION:' in s:raise SystemExit('already generated')
s=s.replace('kind <= ARB1_KIND_DRIVE_INTENT','kind <= ARB1_KIND_CLOCK_ANCHOR')
identity=[('session',0,8),('run',8,8),('nav_epoch',32,8),('map_epoch',40,8),('map_revision',48,8),('localization_epoch',56,8),('plan_id',64,8),('frame_token',72,4)]
helper='''Identity read_identity(const uint8_t* p) {
  Identity i;copy_id(p+16,i.goal);
'''+''.join(f'  i.{n}=u{w*8}(p+{o});\n' for n,o,w in identity)+'''  return i;
}
void write_identity(Bytes& b,const Identity& i) {
  put_id(b,16,i.goal);
'''+''.join(f'  put(b,{o},i.{n},{w});\n' for n,o,w in identity)+''' }
int32_t signed32(const uint8_t* p){uint32_t v=u32(p);int32_t x;std::memcpy(&x,&v,4);return x;}
'''
s=s.replace('bool decode_payload(',helper+'\nbool decode_payload(')
mission=[('command_id',80,8),('gateway_generation',88,8),('wire_request_id',96,8),('operation',104,2),('status',106,2),('reason',108,4),('goal_x_mm',112,-4),('goal_y_mm',116,-4),('start_pose_seq',120,8),('permit_seq',128,8),('auth_epoch',136,8),('produced_mono_ns',144,8),('valid_for_ms',152,4),('flags',156,4),('expected_state_seq',160,8)]
state=[('state_seq',80,8),('pose_seq',88,8),('x_mm',96,-4),('y_mm',100,-4),('yaw_urad',104,-4),('pose_age_ms',108,4),('state',112,1),('avoid_phase',113,1),('reason',114,2),('state_flags',116,4),('cursor',120,2),('count',122,2),('remaining_mm',124,4),('permit_ms',128,4),('auth_ms',132,4),('active_crc',136,4),('pending_crc',140,4),('upload_crc',144,4),('hw_gates',148,4),('pending_epoch',152,8),('pending_revision',160,8),('upload_epoch',168,8),('upload_revision',176,8),('last_ack_seq',184,8),('device_ms',192,4),('owner',196,4),('chunks',200,2),('diagnostic',204,4),('last_request_id',208,8),('pending_frame_token',216,4),('context_flags',220,4),('origin_x_mm',288,-4),('origin_y_mm',292,-4),('resolution_mm',296,2),('width',298,1),('height',299,1)]
plan=[('command_id',80,8),('start_pose_seq',88,8),('path_revision',96,4),('status',100,2),('reason',102,2),('expanded',104,4),('cycles',108,4),('result_flags',116,4),('origin_x_mm',128,-4),('origin_y_mm',132,-4),('resolution_mm',136,2),('width',138,1),('height',139,1),('bitmap_crc32',140,4)]
clock=[('session',0,8),('anchor_id',8,8),('info_request_id',16,8),('host_send_ns',24,8),('host_receive_ns',32,8),('device_ms',40,4),('rtt_bound_ms',44,4),('anchor_request_id',48,8),('anchor_ack_device_ms',56,4),('flags',60,4)]
models=[('Mission','MISSION',176,mission),('FpgaState','FPGA_STATE',304,state),('PlanResult','PLAN_RESULT',144,plan),('ClockAnchor','CLOCK_ANCHOR',64,clock)]
decode=[];encode=[];kinds=[]
for typ,kind,size,fields in models:
 d=f'    case ARB1_KIND_{kind}: {{\n'
 d+=f'      if(n{"<" if typ=="PlanResult" else "!="}{size})return bad("{kind} length");\n      {typ} v;\n'
 if typ!='ClockAnchor':d+='      if(u32(p+76))return bad("identity reserved");v.identity=read_identity(p);\n'
 for name,off,width in fields:
  get=f'signed32(p+{off})' if width<0 else (f'p[{off}]' if width==1 else f'u{width*8}(p+{off})')
  d+=f'      v.{name}={get};\n'
 if typ=='Mission':d+='''      const unsigned op=v.operation&0x7fffu;
      if(u64(p+168)||op<1||op>5||(v.flags&~1u)||v.valid_for_ms>500||!v.command_id||!v.gateway_generation)return bad("MISSION fields");
      if(!(v.operation&0x8000u)&&(v.status||v.wire_request_id))return bad("MISSION request status/id");
      if((op==4||op==5)&&(v.goal_x_mm||v.goal_y_mm||v.permit_seq||v.auth_epoch))return bad("mapping is not a motion ticket");
'''
 elif typ=='FpgaState':d+='''      if(u16(p+202)||u32(p+300)||v.state>5||v.avoid_phase>3||v.count>4096||v.cursor>v.count||v.owner>2||v.chunks>16)return bad("STATE fields");
      if(!read_frame(p+224,v.frame_id))return frame_error();
      if(v.width!=64||v.height!=64||v.resolution_mm!=100)return bad("STATE geometry");
'''
 elif typ=='PlanResult':d+='''      const auto count=u32(p+112);
      if(count>4096||u32(p+120)!=count*2||u32(p+124)||n!=144+count*2||v.width!=64||v.height!=64||v.resolution_mm!=100)return bad("PLAN fields");
      for(unsigned i=0;i<count;++i){auto cell=u16(p+144+2*i);if(cell>=4096)return bad("PLAN cell");v.cells.push_back(cell);}
'''
 else:d+='''      if((v.flags&~1u)||v.host_receive_ns<v.host_send_ns||v.rtt_bound_ms>100)return bad("ANCHOR fields");
'''
 d+='      out=std::move(v);return true;\n    }\n';decode.append(d)
 e=f'    }} else if constexpr (std::is_same_v<T,{typ}>) {{\n      b.resize({size}'+('+v.cells.size()*2' if typ=='PlanResult' else '')+');\n'
 if typ!='ClockAnchor':e+='      write_identity(b,v.identity);\n'
 for name,off,width in fields:e+=f'      put(b,{off},v.{name},{abs(width)});\n'
 if typ=='FpgaState':e+='      if(!write_frame(b,224,v.frame_id))return frame_error();\n'
 if typ=='PlanResult':e+='''      if(v.cells.size()>4096)return fail(e,ErrorCode::Bound,0,"PLAN cells");
      put(b,112,v.cells.size(),4);put(b,120,2*v.cells.size(),4);
      for(size_t i=0;i<v.cells.size();++i)put(b,144+2*i,v.cells[i],2);
'''
 encode.append(e)
 kinds.append(f'    else if constexpr(std::is_same_v<T,{typ}>) return ARB1_KIND_{kind};\n')
s=s.replace('    default: return bad("internal unknown kind");',''.join(decode)+'    default: return bad("internal unknown kind");')
s=s.replace('    } else {\n      if(v.bytes.size()>ARB1_MAX_MESSAGE_BYTES)', ''.join(encode)+'    } else {\n      if(v.bytes.size()>ARB1_MAX_MESSAGE_BYTES)')
s=s.replace('    else return v.kind;',''.join(kinds)+'    else return v.kind;')
s=s.replace('type>ARB1_MSG_DRIVE_INTENT','type>ARB1_MSG_MISSION')
s=s.replace('(h.type==ARB1_MSG_SNAPSHOT && (d.kind<ARB1_KIND_MAP || d.kind>ARB1_KIND_PATH))','(h.type==ARB1_MSG_MISSION && (d.kind!=ARB1_KIND_MISSION || !(d.metadata.flags&ARB1_BLOCK_REQUIRED))) ||\n         (h.type==ARB1_MSG_SNAPSHOT && !((d.kind>=ARB1_KIND_MAP&&d.kind<=ARB1_KIND_PATH)||(d.kind>=ARB1_KIND_FPGA_STATE&&d.kind<=ARB1_KIND_CLOCK_ANCHOR)))')
pos=s.rfind('}  // namespace arb1')
s=s[:pos]+'''bool executable(const Mission& v,uint64_t now_ns) {
  return !(v.operation&0x8000u)&&!(v.flags&ARB1_MISSION_DRY_RUN)&&v.valid_for_ms>0&&v.valid_for_ms<=500&&v.produced_mono_ns<=now_ns&&now_ns-v.produced_mono_ns<uint64_t(v.valid_for_ms)*1000000ull;
}
'''+s[pos:]
p.write_text(s)
