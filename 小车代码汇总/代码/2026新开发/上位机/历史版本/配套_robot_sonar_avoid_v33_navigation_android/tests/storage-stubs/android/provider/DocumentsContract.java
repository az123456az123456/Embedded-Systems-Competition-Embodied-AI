package android.provider;import android.net.Uri;import android.content.ContentResolver;import java.io.IOException;
/** Bounded host dispatch adapter; real Android implementation used in APK. */
public final class DocumentsContract {
 public static final class Document {public static final String COLUMN_DISPLAY_NAME="name",COLUMN_DOCUMENT_ID="id",COLUMN_MIME_TYPE="mime",MIME_TYPE_DIR="vnd.android.document/directory";}
 public static boolean isTreeUri(Uri uri){return uri.toString().contains("/tree/");}
 public static String getTreeDocumentId(Uri uri){String[] p=uri.toString().split("/tree/",2);return p[1].split("/")[0];}
 public static String getDocumentId(Uri uri){return uri.toString().split("/document/",2)[1].split("/")[0];}
 public static Uri buildDocumentUriUsingTree(Uri tree,String id){return Uri.parse(tree.toString().split("/tree/",2)[0]+"/tree/"+getTreeDocumentId(tree)+"/document/"+id);}
 public static Uri buildChildDocumentsUriUsingTree(Uri tree,String id){return Uri.parse(buildDocumentUriUsingTree(tree,id)+"/children");}
 public static Uri createDocument(ContentResolver resolver,Uri parent,String mime,String name)throws IOException{return resolver.createDocument(parent,mime,name);}
 public static boolean deleteDocument(ContentResolver resolver,Uri uri)throws IOException{return resolver.deleteDocument(uri);}
}
