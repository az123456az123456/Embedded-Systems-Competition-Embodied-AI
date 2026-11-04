package android.content;public class Context {
 private final ContentResolver resolver;public Context(ContentResolver resolver){this.resolver=resolver;}
 public Context getApplicationContext(){return this;}public ContentResolver getContentResolver(){return resolver;}
}
