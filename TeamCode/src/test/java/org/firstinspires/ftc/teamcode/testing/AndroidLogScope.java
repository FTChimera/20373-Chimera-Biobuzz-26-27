package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import org.mockito.MockedStatic;
import org.mockito.Mockito;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Records Android/SDK logging on the test thread instead of calling Android's log transport. */
public final class AndroidLogScope implements AutoCloseable {
    public static final class Entry {
        public final int priority;
        public final String tag;
        public final String message;

        private Entry(int priority, String tag, String message) {
            this.priority = priority;
            this.tag = tag;
            this.message = message;
        }
    }

    private final List<Entry> entries = new ArrayList<>();
    private final MockedStatic<Log> logging;

    public AndroidLogScope() {
        logging = Mockito.mockStatic(Log.class, invocation -> {
            String method = invocation.getMethod().getName();
            Object[] args = invocation.getArguments();
            if (method.equals("isLoggable")) return true;
            if (method.equals("getStackTraceString")) return stackTrace((Throwable) args[0]);
            if (method.equals("println")) {
                return record((Integer) args[0], (String) args[1], (String) args[2]);
            }
            int priority;
            switch (method) {
                case "v": priority = Log.VERBOSE; break;
                case "d": priority = Log.DEBUG; break;
                case "i": priority = Log.INFO; break;
                case "w": priority = Log.WARN; break;
                case "e": priority = Log.ERROR; break;
                case "wtf": priority = Log.ASSERT; break;
                default: throw new UnsupportedOperationException("Unsupported Android logging call: "
                        + invocation.getMethod());
            }
            String message = args[1] instanceof Throwable
                    ? stackTrace((Throwable) args[1]) : String.valueOf(args[1]);
            if (args.length == 3 && args[2] != null) {
                message += "\n" + stackTrace((Throwable) args[2]);
            }
            return record(priority, (String) args[0], message);
        });
    }

    public List<Entry> entries() {
        return Collections.unmodifiableList(new ArrayList<>(entries));
    }

    private int record(int priority, String tag, String message) {
        if (message == null) throw new IllegalArgumentException("Log message must not be null");
        entries.add(new Entry(priority, tag, message));
        return message.getBytes(StandardCharsets.UTF_8).length;
    }

    private static String stackTrace(Throwable error) {
        if (error == null) return "";
        StringWriter writer = new StringWriter();
        error.printStackTrace(new PrintWriter(writer));
        return writer.toString();
    }

    @Override public void close() { logging.close(); }
}
