package cn.acg138.robotphone;

import android.content.ContentResolver;
import android.content.Context;
import android.content.UriPermission;
import android.database.Cursor;
import android.net.Uri;
import android.provider.DocumentsContract;
import java.io.IOException;
import java.io.OutputStream;

/** Uses only the user's selected document tree; no broad storage permission. */
public final class SafCaptureStore implements CaptureArchive.Store {
    private final ContentResolver resolver;
    private final Uri tree;
    public SafCaptureStore(Context context, Uri tree) {
        this.resolver = context.getApplicationContext().getContentResolver(); this.tree = tree;
        if (!hasGrant(resolver, tree)) throw new IllegalArgumentException("截图目录授权已失效，请重新选择目录");
    }
    public static boolean hasGrant(ContentResolver resolver, Uri tree) {
        if (tree == null || !"content".equals(tree.getScheme()) || !DocumentsContract.isTreeUri(tree)) return false;
        for (UriPermission permission : resolver.getPersistedUriPermissions())
            if (tree.equals(permission.getUri()) && permission.isReadPermission() && permission.isWritePermission()) return true;
        return false;
    }
    public static String describe(Context context, Uri tree) {
        try {
            ContentResolver resolver = context.getContentResolver();
            Uri document = DocumentsContract.buildDocumentUriUsingTree(tree, DocumentsContract.getTreeDocumentId(tree));
            try (Cursor cursor = resolver.query(document, new String[]{DocumentsContract.Document.COLUMN_DISPLAY_NAME}, null, null, null)) {
                if (cursor != null && cursor.moveToFirst()) return cursor.getString(0);
            }
        } catch (RuntimeException ignored) { }
        return tree == null ? "未选择目录" : tree.toString();
    }
    private Uri child(Uri parent, String name) throws IOException {
        Uri children = DocumentsContract.buildChildDocumentsUriUsingTree(tree, DocumentsContract.getDocumentId(parent));
        try (Cursor cursor = resolver.query(children, new String[]{DocumentsContract.Document.COLUMN_DOCUMENT_ID,
                DocumentsContract.Document.COLUMN_DISPLAY_NAME, DocumentsContract.Document.COLUMN_MIME_TYPE}, null, null, null)) {
            if (cursor == null) throw new IOException("无法读取所选目录");
            while (cursor.moveToNext()) if (name.equals(cursor.getString(1)))
                return DocumentsContract.buildDocumentUriUsingTree(tree, cursor.getString(0));
        }
        return null;
    }
    private String displayName(Uri uri) throws IOException {
        try (Cursor cursor = resolver.query(uri, new String[]{DocumentsContract.Document.COLUMN_DISPLAY_NAME}, null, null, null)) {
            if (cursor == null || !cursor.moveToFirst()) throw new IOException("无法确认已保存的文件名");
            return cursor.getString(0);
        }
    }
    private boolean directory(Uri uri) throws IOException {
        try (Cursor cursor = resolver.query(uri, new String[]{DocumentsContract.Document.COLUMN_MIME_TYPE}, null, null, null)) {
            if (cursor == null || !cursor.moveToFirst()) throw new IOException("日期目录无法读取");
            return DocumentsContract.Document.MIME_TYPE_DIR.equals(cursor.getString(0));
        }
    }
    private Uri dayDirectory(String name) throws IOException {
        if (!hasGrant(resolver, tree)) throw new IOException("截图目录授权已失效，请在设置中重新选择");
        if (!name.matches("[0-9]{4}-[0-9]{2}-[0-9]{2}")) throw new IOException("日期目录名无效");
        Uri parent = DocumentsContract.buildDocumentUriUsingTree(tree, DocumentsContract.getTreeDocumentId(tree));
        Uri day = child(parent, name);
        if (day == null) day = DocumentsContract.createDocument(resolver, parent, DocumentsContract.Document.MIME_TYPE_DIR, name);
        if (day == null || !directory(day) || !name.equals(displayName(day))) throw new IOException("无法创建日期目录 " + name);
        return day;
    }
    @Override public void ensureDay(String day) throws IOException { dayDirectory(day); }
    @Override public String write(CaptureArchive.Stamp stamp, byte[] jpeg) throws Exception {
        if (!stamp.filename.matches("[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{3}_frame[0-9]+[.]jpg")) throw new IOException("截图文件名无效");
        Uri day = dayDirectory(stamp.day);
        // A clock adjustment or board restart can repeat time/frame IDs. Do not
        // open an existing image: create a unique suffix instead of overwriting it.
        String filename = stamp.filename;
        for (int suffix = 2; child(day, filename) != null; suffix++) {
            if (suffix > 10000) throw new IOException("截图同名文件过多");
            filename = stamp.filename.substring(0, stamp.filename.length() - 4) + "_" + suffix + ".jpg";
        }
        Uri image = DocumentsContract.createDocument(resolver, day, "image/jpeg", filename);
        if (image == null) throw new IOException("无法创建截图文件，请检查可用空间");
        boolean complete = false;
        try {
            try (OutputStream stream = resolver.openOutputStream(image, "w")) {
                if (stream == null) throw new IOException("无法写入截图文件");
                stream.write(jpeg); stream.flush();
            }
            String actualName = displayName(image);
            complete = true; return stamp.day + "/" + actualName;
        } finally {
            if (!complete) try { DocumentsContract.deleteDocument(resolver, image); } catch (Exception ignored) { }
        }
    }
}
