from pathlib import Path
import hashlib, json, subprocess
root=Path(__file__).resolve().parent
workspace=root.parents[1]
dashboard=workspace/'rk3588/robot_control_v39/dashboard/src'
build=root/'build'; build.mkdir(exist_ok=True)
subprocess.run(['c++','-std=c++17','-O2','-fPIC','-shared','-I'+str(dashboard),
                str(root/'decode_reference.cc'),str(dashboard/'detector_math.cc'),
                '-o',str(build/'libdecode_reference.so')],check=True)
compiler=next(workspace.glob('bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++'))
sdk=workspace/'YOLO/rknn_model_zoo/3rdparty'
subprocess.run([str(compiler),'-std=c++17','-O2','-Wall','-Wextra','-pthread',
                '-static-libstdc++','-static-libgcc','-DHAVE_RKNN',
                '-I'+str(dashboard),'-I'+str(sdk/'rknpu2/include'),'-I'+str(sdk/'stb_image'),
                str(root/'npu_model_check.cc'),str(dashboard/'detector.cc'),str(dashboard/'detector_math.cc'),
                '-L'+str(sdk/'rknpu2/Linux/aarch64'),'-lrknnrt','-ldl','-lm',
                '-Wl,--allow-shlib-undefined','-Wl,-rpath,/root/robot-perception/lib',
                '-o',str(build/'npu-model-check.aarch64')],check=True)
inputs=[root/'decode_reference.cc',root/'npu_model_check.cc',dashboard/'detector.cc',dashboard/'detector_math.cc']
(root/'reports/check-build.json').write_text(json.dumps({'status':'PASS','inputs':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},'motor_or_network_io':False},indent=2)+'\n')
