package org.json;
import java.util.LinkedHashMap;
/** Bounded host adapter only; never packaged into the APK. */
public class JSONObject {
    private final LinkedHashMap<String,Object> values = new LinkedHashMap<>();
    public JSONObject() { }
    public JSONObject(String source) {
        Parser parser = new Parser(source); JSONObject parsed = parser.object(); parser.space();
        if (parser.at != source.length()) throw new IllegalArgumentException("trailing JSON");
        values.putAll(parsed.values);
    }
    public JSONObject put(String key,Object value){values.put(key,value);return this;}
    public Object remove(String key){return values.remove(key);}
    public Object opt(String key){return values.get(key);}
    public boolean has(String key){return values.containsKey(key);}
    public JSONObject optJSONObject(String key){Object v=values.get(key);return v instanceof JSONObject?(JSONObject)v:null;}
    public JSONArray optJSONArray(String key){Object v=values.get(key);return v instanceof JSONArray?(JSONArray)v:null;}
    public static volatile Runnable beforeArmedBooleanForTest;
    public boolean optBoolean(String key){return optBoolean(key,false);}
    public boolean optBoolean(String key,boolean fallback){Runnable hook=beforeArmedBooleanForTest;if("armed".equals(key)&&hook!=null)hook.run();Object v=values.get(key);return v instanceof Boolean?(Boolean)v:fallback;}
    public long optLong(String key){return optLong(key,0);}
    public long optLong(String key,long fallback){Object v=values.get(key);return v instanceof Number?((Number)v).longValue():fallback;}
    public int optInt(String key,int fallback){return (int)optLong(key,fallback);}
    public double optDouble(String key,double fallback){Object v=values.get(key);return v instanceof Number?((Number)v).doubleValue():fallback;}
    public double optDouble(String key){Object v=values.get(key);return v instanceof Number?((Number)v).doubleValue():0;}
    public String optString(String key){return optString(key,"");}
    public String optString(String key,String fallback){Object v=values.get(key);return v instanceof String?(String)v:fallback;}
    public String getString(String key){if(!values.containsKey(key))throw new IllegalArgumentException(key);return optString(key);}
    static String encode(Object value){
        if(value==null)return "null";
        if(value instanceof String)return "\""+((String)value).replace("\\","\\\\").replace("\"","\\\"").replace("\n","\\n")+"\"";
        return value.toString();
    }
    @Override public String toString(){StringBuilder b=new StringBuilder("{");for(java.util.Map.Entry<String,Object> e:values.entrySet()){if(b.length()>1)b.append(',');b.append(encode(e.getKey())).append(':').append(encode(e.getValue()));}return b.append('}').toString();}
    static class Parser {
        final String source;int at;Parser(String s){source=s;}
        void space(){while(at<source.length()&&Character.isWhitespace(source.charAt(at)))at++;}
        void take(char c){space();if(at>=source.length()||source.charAt(at++)!=c)throw new IllegalArgumentException("JSON token "+c);}
        char peek(){space();return at<source.length()?source.charAt(at):'\0';}
        String string(){take('"');StringBuilder b=new StringBuilder();while(at<source.length()){char c=source.charAt(at++);if(c=='"')return b.toString();if(c=='\\'){c=source.charAt(at++);if(c=='n')c='\n';if(c=='r')c='\r';if(c=='t')c='\t';}b.append(c);}throw new IllegalArgumentException("JSON string");}
        JSONObject object(){take('{');JSONObject o=new JSONObject();if(peek()=='}'){at++;return o;}for(;;){String key=string();take(':');o.put(key,value());if(peek()=='}'){at++;return o;}take(',');}}
        Object value(){char c=peek();if(c=='{')return object();if(c=='"')return string();if(c=='['){at++;java.util.ArrayList<Object> a=new java.util.ArrayList<>();if(peek()!=']')for(;;){a.add(value());if(peek()==']')break;take(',');}take(']');return new JSONArray(a.toArray());}if(source.startsWith("true",at)){at+=4;return true;}if(source.startsWith("false",at)){at+=5;return false;}if(source.startsWith("null",at)){at+=4;return null;}int begin=at;while(at<source.length()&&"0123456789+-.eE".indexOf(source.charAt(at))>=0)at++;if(at==begin)throw new IllegalArgumentException("JSON value");return Double.valueOf(source.substring(begin,at));}
    }
}
