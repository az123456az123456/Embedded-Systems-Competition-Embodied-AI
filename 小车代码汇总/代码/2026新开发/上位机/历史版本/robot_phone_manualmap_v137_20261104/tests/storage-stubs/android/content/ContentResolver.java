package android.content;import android.net.Uri;import android.database.Cursor;import java.io.*;import java.util.List;
public abstract class ContentResolver {
 public abstract List<UriPermission> getPersistedUriPermissions();public abstract Cursor query(Uri uri,String[] projection,String selection,String[] args,String order);
 public abstract OutputStream openOutputStream(Uri uri,String mode)throws IOException;
 public abstract Uri createDocument(Uri parent,String mime,String name)throws IOException;public abstract boolean deleteDocument(Uri uri)throws IOException;
}
