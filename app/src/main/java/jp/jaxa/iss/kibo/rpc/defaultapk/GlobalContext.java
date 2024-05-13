package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.Context;
import android.util.Log;

public class GlobalContext {
    private static GlobalContext instance = null;
    private final Context applicationContext;

    private GlobalContext(Context applicationContext) {
        this.applicationContext = applicationContext;
    }

    // publicをつけないのは意図的
    // MyApplicationと同じパッケージにして、このメソッドのアクセスレベルはパッケージローカルとする
    // 念のため意図しないところで呼び出されることを防ぐため
    static void onCreateApplication(Context applicationContext) {
        // Application#onCreateのタイミングでシングルトンが生成される
        instance = new GlobalContext(applicationContext);
        Log.i("StellarCoders[GlobalContext]", "applicationContext initialized");
    }

    public static Context getInstance() {
        if (instance == null) {
            // こんなことは起きないはず
            throw new RuntimeException("MyContext should be initialized!");
        }
        return instance.applicationContext;
    }
}
