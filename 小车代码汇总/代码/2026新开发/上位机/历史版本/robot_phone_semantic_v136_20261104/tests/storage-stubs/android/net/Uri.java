package android.net;
public final class Uri {
 private final String value;private Uri(String value){this.value=value;}
 public static Uri parse(String value){return new Uri(value);}public String toString(){return value;}public String getScheme(){int i=value.indexOf(':');return i<0?null:value.substring(0,i);}
 @Override public boolean equals(Object o){return o instanceof Uri&&value.equals(((Uri)o).value);}@Override public int hashCode(){return value.hashCode();}
}
