from pathlib import Path
import hashlib,html,re

def table_rows(text):
    return [[html.unescape(re.sub("<[^>]+>", "", cell)).strip()
             for cell in re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", row, re.S)]
            for row in re.findall(r"<tr[^>]*>(.*?)</tr>", text, re.S)]


def named_section(text, start, end):
    a=text.index('name="'+start+'"')
    b=text.index('name="'+end+'"',a+1)
    return text[a:b]


def timing_report(stage):
    paths=list(stage.glob("impl/pnr/*_tr_content.html"))
    if not paths:
        return {"result":"PENDING", "scope":"PnR/STA report not generated yet"}
    path=paths[0]
    text=path.read_text()
    if "</html>" not in text:
        return {"result":"PENDING", "scope":"STA report still being written"}
    summary={row[0]:row[1] for row in table_rows(named_section(text,'STA_Tool_Run_Summary','Clock_Report')) if len(row)==2}
    report={"path":str(path),"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "setup_violated_endpoints":int(summary['Numbers of Setup Violated Endpoints']),
            "hold_violated_endpoints":int(summary['Numbers of Hold Violated Endpoints']),
            "scope":"Declared internal clocks only; camera/RGMII physical external I/O not board signed off"}
    report['negative_paths']={}
    for kind,end in zip(('Setup','Hold','Recovery','Removal'),('Hold_Slack_Table','Recovery_Slack_Table','Removal_Slack_Table','MIN_PULSE_WIDTH_TABLE')):
        rows=[row for row in table_rows(named_section(text,kind+'_Slack_Table',end)) if len(row)>=4 and row[0].isdigit()]
        report[kind.lower()+'_reported_paths']=len(rows)
        report[kind.lower()+'_minimum_slack_ns']=min(float(row[1]) for row in rows)
        report['negative_paths'][kind]=[dict(slack_ns=float(row[1]),source=row[2],destination=row[3],row=row) for row in rows if float(row[1])<0]
    report['result']='PASS' if report['setup_violated_endpoints']==report['hold_violated_endpoints']==0 and not any(report['negative_paths'].values()) else 'FAIL'
    return report

