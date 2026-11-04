import android.content.*;import android.net.Uri;import android.database.Cursor;import android.provider.DocumentsContract;
import cn.acg138.robotphone.*;import java.util.*;import java.io.*;

/** Production SAF writer executed against a bounded in-memory document provider. */
public final class SafStoreHostTests {
 static int checks;static void check(boolean value,String reason){if(!value)throw new AssertionError(reason);checks++;}
 static class Node {String id,name,mime,parent;byte[] bytes;Node(String id,String name,String mime,String parent){this.id=id;this.name=name;this.mime=mime;this.parent=parent;}}
 static class Provider extends ContentResolver {
  final LinkedHashMap<String,Node> nodes=new LinkedHashMap<>();final List<UriPermission> grants=new ArrayList<>();final Uri tree=Uri.parse("content://test/tree/root");int next;boolean failOpen,failClose;int deleted;
  Provider(){nodes.put("root",new Node("root","Robot photos",DocumentsContract.Document.MIME_TYPE_DIR,null));grants.add(new UriPermission(tree,true,true));}
  String id(Uri uri){return DocumentsContract.getDocumentId(uri);}Node node(Uri uri){return nodes.get(id(uri));}
  public List<UriPermission> getPersistedUriPermissions(){return grants;}
  public Cursor query(Uri uri,String[] projection,String selection,String[] args,String order){
   ArrayList<String[]> rows=new ArrayList<>();String parent=id(uri);if(!nodes.containsKey(parent))return null;
   for(Node n:nodes.values())if(uri.toString().endsWith("/children")?parent.equals(n.parent):parent.equals(n.id)){
    String[] row=new String[projection.length];for(int i=0;i<projection.length;i++)row[i]="name".equals(projection[i])?n.name:"mime".equals(projection[i])?n.mime:n.id;rows.add(row);
   }
   return new Cursor(){int at=-1;public boolean moveToFirst(){at=0;return at<rows.size();}public boolean moveToNext(){at++;return at<rows.size();}public String getString(int c){return rows.get(at)[c];}public void close(){}};
  }
  public Uri createDocument(Uri parent,String mime,String name)throws IOException{
   if(!nodes.containsKey(id(parent)))throw new IOException("missing parent");String id="n"+(++next);nodes.put(id,new Node(id,name,mime,id(parent)));return DocumentsContract.buildDocumentUriUsingTree(tree,id);
  }
  public boolean deleteDocument(Uri uri){deleted++;return nodes.remove(id(uri))!=null;}
  public OutputStream openOutputStream(Uri uri,String mode)throws IOException {
   if(failOpen)throw new IOException("no space");Node target=node(uri);if(target==null)throw new IOException("missing image");
   return new ByteArrayOutputStream(){@Override public void close()throws IOException{if(failClose)throw new IOException("close failed");target.bytes=toByteArray();}};
  }
  Node named(String parent,String name){for(Node n:nodes.values())if(parent.equals(n.parent)&&name.equals(n.name))return n;return null;}
 }
 static void expectedFailure(Throwing action,String reason)throws Exception{try{action.run();throw new AssertionError(reason);}catch(IOException|IllegalArgumentException expected){checks++;}}
 interface Throwing{void run()throws Exception;}
 public static void main(String[] args)throws Exception{
  Provider provider=new Provider();Context context=new Context(provider);Uri tree=provider.tree;
  check(SafCaptureStore.hasGrant(provider,tree),"persisted read+write grant recognized");check("Robot photos".equals(SafCaptureStore.describe(context,tree)),"user-facing actual selected directory label");
  check(!SafCaptureStore.hasGrant(provider,Uri.parse("content://test/tree/other")),"another tree never inherits grant");
  provider.grants.clear();provider.grants.add(new UriPermission(tree,true,false));check(!SafCaptureStore.hasGrant(provider,tree),"read-only tree is not writable archive");expectedFailure(()->new SafCaptureStore(context,tree),"constructor requires selected persistent write grant");
  provider.grants.clear();provider.grants.add(new UriPermission(tree,true,true));SafCaptureStore store=new SafCaptureStore(context,tree);
  long wall=java.time.Instant.parse("2026-10-26T15:59:59.999Z").toEpochMilli();byte[] bytes={(byte)255,(byte)216,1,2,3,(byte)255,(byte)217};
  CaptureArchive.Stamp a=CaptureArchive.Stamp.at(wall,TimeZone.getTimeZone("Asia/Shanghai"),90);
  store.ensureDay("2026-10-26");check(provider.nodes.size()==2&&provider.named("root","2026-10-26")!=null,"foreground ensure creates real date folder before any no_vest/photo");
  store.ensureDay("2026-10-26");check(provider.nodes.size()==2,"ensure existing date creates no duplicate directory");
  check(provider.nodes.values().stream().allMatch(n->n.bytes==null),"ensure creates no placeholder image");
  expectedFailure(()->store.ensureDay("../escape"),"date ensure rejects path traversal");
  String first=store.write(a,bytes);Node day=provider.named("root","2026-10-26");check(day!=null&&DocumentsContract.Document.MIME_TYPE_DIR.equals(day.mime),"date directory created inside selected tree");
  check(first.equals("2026-10-26/23-59-59-999_frame90.jpg")&&Arrays.equals(bytes,provider.named(day.id,a.filename).bytes),"exact camera JPEG stored under phone date/time");
  String second=store.write(a,bytes);check(second.endsWith("_frame90_2.jpg")&&provider.named(day.id,a.filename).bytes.length==bytes.length,"clock/frame collision creates suffix without overwriting");
  CaptureArchive.Stamp b=CaptureArchive.Stamp.at(wall+1,TimeZone.getTimeZone("Asia/Shanghai"),91);check(store.write(b,bytes).startsWith("2026-10-27/")&&provider.named("root","2026-10-27")!=null,"next phone date creates new directory");
  int original=provider.nodes.size();provider.failOpen=true;expectedFailure(()->store.write(a,bytes),"failed open surfaces no space");check(provider.nodes.size()==original&&provider.deleted==1,"failed image cleaned up without deleting existing files");
  provider.failOpen=false;provider.failClose=true;expectedFailure(()->store.write(a,bytes),"failed close is not reported success");check(provider.nodes.size()==original&&provider.deleted==2,"incomplete close removes only newly created image");provider.failClose=false;
  provider.grants.clear();expectedFailure(()->store.write(a,bytes),"revoked grant fails clearly at write time");check(provider.nodes.size()==original,"revoked grant cannot create anything");
  provider.grants.add(new UriPermission(tree,true,true));CaptureArchive.Stamp c=CaptureArchive.Stamp.at(wall+86400001L,TimeZone.getTimeZone("Asia/Shanghai"),92);provider.createDocument(DocumentsContract.buildDocumentUriUsingTree(tree,"root"),"text/plain",c.day);
  expectedFailure(()->store.write(c,bytes),"a normal file named like the date cannot be overwritten as a folder");
  check(!SafCaptureStore.hasGrant(provider,Uri.parse("file:///tmp/photos")),"ungranted raw filesystem path rejected");
  System.out.println("PASS "+checks+" production SAF writer checks with in-memory provider; real Android picker/provider validation pending");
 }
}
