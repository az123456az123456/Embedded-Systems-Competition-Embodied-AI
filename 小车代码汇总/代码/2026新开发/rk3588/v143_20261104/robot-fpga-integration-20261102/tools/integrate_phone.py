from pathlib import Path
r=Path('rk3588/robot_phone_semantic_v135_20261103')
def edit(name,old,new):
 p=r/name;s=p.read_text();assert old in s,(name,old[:80]);p.write_text(s.replace(old,new))
edit('AndroidManifest.xml','android:versionCode="47"','android:versionCode="48"')
edit('AndroidManifest.xml','android:versionName="1.2.34"','android:versionName="1.2.35"')
D='src/cn/acg138/robotphone/DashboardView.java'
edit(D,'void startNavigation(); void startMapping();','void semanticInput(); void startSemantic();\n        void startNavigation(); void startMapping();')
edit(D,'    private String destinationStatus = "";','    private String destinationStatus = "";\n    private String semanticText="";\n    public void semanticText(String text){semanticText=text==null?"":text;invalidate();}')
edit(D,'String display = tab == 1 && mappingFresh && MappingPresentation.loaded(state,navigationMap)',
 'String semantic=mappingFresh?SemanticPresentation.status(state):"";\n        String display = !semantic.isEmpty()?semantic:tab == 1 && mappingFresh && MappingPresentation.loaded(state,navigationMap)')
edit(D,'choiceButton(c,x,138,bw,49,"arm",armed?"停用电机":"使能电机");',
 'choiceButton(c,x,132,bw,43,"arm",armed?"停用电机":"使能电机");')
edit(D,'choiceButton(c,x+bw+10,138,bw,49,"nav_start",navigationButtonLabel());',
 'choiceButton(c,x+bw+10,132,bw,43,"nav_start",navigationButtonLabel());')
edit(D,'choiceButton(c,x,197,bw,49,"mapping",mappingButtonLabel());',
 'choiceButton(c,x,183,bw,43,"mapping",mappingButtonLabel());')
edit(D,'choiceButton(c,x+bw+10,197,bw,49,"nav_end","结束导航");',
 'choiceButton(c,x+bw+10,183,bw,43,"nav_end","结束导航");')
edit(D,'        text(c,"天蓝路线 · 灰色轨迹 · 摇杆接管",x,263,12,muted);',
 '''        float inputWidth=rightWidth-134;
        choiceButton(c,x,235,inputWidth,27,"semantic_input",fit(semanticText.isEmpty()?"输入搜索目标…":semanticText,inputWidth-12,13));
        choiceButton(c,x+inputWidth+8,235,98,27,"semantic_start",SemanticPresentation.active(state)?"结束搜索":"开始搜索");''')
edit(D,'            case "settings": clearGesture(); actions.settings(); break;',
 '''            case "semantic_input": clearGesture(); actions.semanticInput(); break;
            case "semantic_start": clearGesture(); if(SemanticPresentation.active(state))actions.stop();else actions.startSemantic(); break;
            case "settings": clearGesture(); actions.settings(); break;''')
A='src/cn/acg138/robotphone/MainActivity.java'
edit(A,'        setContentView(dashboard);','        setContentView(dashboard);\n        dashboard.semanticText(settings.getString("semantic_command",""));')
edit(A,'    @Override public void startMapping() {',
 '''    @Override public void semanticInput() {
        if(client!=null)client.pauseMotion("semantic_input_open");
        LinearLayout form=new LinearLayout(this);form.setOrientation(LinearLayout.VERTICAL);form.setPadding(dp(20),dp(8),dp(20),dp(8));
        addLabel(form,"输入目标后点确定，再点“开始搜索”。需先使能电机。");
        EditText input=new EditText(this);input.setSingleLine();input.setHint("例如：找到未戴口罩的人");
        input.setText(settings.getString("semantic_command",""));styleInput(input);form.addView(input);
        addLabel(form,"支持：未戴口罩、未戴安全帽、未穿反光衣的人，以及人员、车辆、机械、安全帽、口罩、反光衣。确认目标后停车，不自动接近。");
        String[] examples={"找到未戴口罩的人","找到未戴安全帽的人","找到未穿反光衣的人","找到人员","找到车辆"};
        for(String example:examples){Button b=new Button(this);b.setText(example);b.setOnClickListener(v -> input.setText(example));form.addView(b);}
        new AlertDialog.Builder(this).setTitle("语义导航目标").setView(form).setNegativeButton("取消",null)
            .setPositiveButton("确定",(dialog,which)->{String text=input.getText().toString().trim();settings.edit().putString("semantic_command",text).apply();dashboard.semanticText(text);}).show();
    }
    @Override public void startSemantic() {
        String text=settings.getString("semantic_command","").trim();
        if(text.isEmpty()){semanticInput();return;}
        if(client!=null){client.startSemantic(text);dashboard.flags(client.isArmed(),client.mode());}
        else dashboard.status("请先连接小车");
    }
    @Override public void startMapping() {''')
C='src/cn/acg138/robotphone/RobotClient.java'
edit(C,'    private volatile boolean mappingStopPending;','    private volatile boolean mappingStopPending;\n    private volatile String semanticCommand="";')
edit(C,'    private void beginNavigation(String operation,NavigationTarget target) {',
 '''    public void startSemantic(String text) {
        if(text==null||text.trim().isEmpty()||text.length()>80){listener.navigation("请输入支持的搜索目标",null);return;}
        if(!motionAllowed()){listener.navigation("请先使能电机并等待状态恢复",null);return;}
        if("mapping".equals(mode)||"navigation".equals(mode)||MappingPresentation.busy(latestState)){
            listener.navigation("请先结束当前任务，再开始语义搜索",null);return;
        }
        synchronized(navigationLock){semanticCommand=text.trim();beginNavigation("semantic-start",null);}
    }
    private void beginNavigation(String operation,NavigationTarget target) {''')
edit(C,'mode="mapping-start".equals(operation)?"mapping":',
 'mode=("mapping-start".equals(operation)||"semantic-start".equals(operation))?"mapping":')
edit(C,'        try { navigation.execute(() -> {\n            if(!currentNavigation',
 '        final String commandText=semanticCommand;\n        try { navigation.execute(() -> {\n            if(!currentNavigation')
edit(C,'                request("POST","/api/navigation",requestBody,1200);',
 '                if("semantic-start".equals(operation))requestBody.put("text",commandText);\n                request("POST","/api/navigation",requestBody,1200);')
edit(C,'listener.navigation("mapping-start".equals(operation)?"探索请求已接收，等待 RK 开始建图":',
 'listener.navigation("semantic-start".equals(operation)?"语义搜索请求已接收，等待 RK 启动":"mapping-start".equals(operation)?"探索请求已接收，等待 RK 开始建图":')
edit(C,'                JSONObject mapping=MappingPresentation.mapping(state);\n                if(mapping!=null) {',
 '''                JSONObject semantic=SemanticPresentation.state(state);
                boolean semanticTask=semantic!=null&&!semantic.optString("task","").isEmpty()&&"semantic-start".equals(navigationOperation);
                if(semanticTask){
                    String status=SemanticPresentation.status(state);
                    if(!status.equals(navigationStatus)){navigationStatus=status;listener.navigation(status,null);}
                    String phase=semantic.optString("state","");
                    if("FOUND".equals(phase)||"NOT_FOUND".equals(phase)||"ERROR".equals(phase)||"CANCELLED".equals(phase)){
                        cancelNavigationIntent();mode="manual";mappingFinishAcceptedAt=0;
                    }
                }
                JSONObject mapping=MappingPresentation.mapping(state);
                if(!semanticTask&&mapping!=null) {''')
edit('tests/GuiHostTests.java','public void startNavigation(){starts++;}',
 'public void semanticInput(){}public void startSemantic(){}public void startNavigation(){starts++;}')
for name in ('test-ui-host.sh','test-client-host.sh'):
 p=r/name;s=p.read_text();needle='"$HERE/src/cn/acg138/robotphone/MappingPresentation.java"'
 if needle in s:s=s.replace(needle,needle+' "$HERE/src/cn/acg138/robotphone/SemanticPresentation.java"');p.write_text(s)
