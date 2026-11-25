package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

/**
 * In-memory FTC telemetry. Every update records a snapshot immediately (no transport throttling).
 * Values are rendered at update time, after actions; duplicate captions and line order are preserved.
 * Auto-clear is deferred until the next addition, matching the SDK. Removed line handles fail clearly.
 * Speech and decimal-place configuration are deliberately unsupported, rather than silently ignored.
 */
public final class RecordingTelemetry implements Telemetry {
    public static final class Entry {
        public final String caption;
        public final String value;

        private Entry(String caption, String value) {
            this.caption = caption;
            this.value = value;
        }
    }

    public static final class Update {
        public final List<Entry> entries;
        public final List<String> log;

        private Update(List<Entry> entries, List<String> log) {
            this.entries = Collections.unmodifiableList(new ArrayList<>(entries));
            this.log = Collections.unmodifiableList(new ArrayList<>(log));
        }

        /** Last value wins for duplicate captions; use entries when duplicates matter. */
        public Map<String, String> values() {
            Map<String, String> values = new LinkedHashMap<>();
            for (Entry entry : entries) values.put(entry.caption, entry.value);
            return Collections.unmodifiableMap(values);
        }
    }

    private final List<Row> rows = new ArrayList<>();
    private final Map<Object, Runnable> actions = new LinkedHashMap<>();
    private final List<Update> updates = new ArrayList<>();
    private final RecordingLog log = new RecordingLog();
    private boolean autoClear = true;
    private boolean clearOnAdd;
    private int interval = 250;
    private String itemSeparator = " | ";
    private String captionSeparator = " : ";
    private DisplayFormat displayFormat = DisplayFormat.CLASSIC;

    public List<Update> updates() {
        return Collections.unmodifiableList(new ArrayList<>(updates));
    }

    public Update latest() {
        if (updates.isEmpty()) throw new IllegalStateException("Telemetry has not been updated");
        return updates.get(updates.size() - 1);
    }

    @Override public Item addData(String caption, Object value) {
        return newRow(null).add(caption, value, null, false);
    }
    @Override public Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(Locale.ROOT, format, args));
    }
    @Override public <T> Item addData(String caption, Func<T> producer) {
        return newRow(null).add(caption, Objects.requireNonNull(producer), null, true);
    }
    @Override public <T> Item addData(String caption, String format, Func<T> producer) {
        return newRow(null).add(caption, Objects.requireNonNull(producer),
                Objects.requireNonNull(format), true);
    }
    @Override public Line addLine() { return addLine(""); }
    @Override public Line addLine(String caption) {
        return newRow(Objects.requireNonNull(caption)).line;
    }
    @Override public boolean removeLine(Line line) {
        return rows.removeIf(row -> row.line == line);
    }
    @Override public boolean removeItem(Item item) {
        for (Row row : new ArrayList<>(rows)) {
            if (row.items.removeIf(data -> data.item == item)) {
                if (row.items.isEmpty() && row.caption == null) rows.remove(row);
                return true;
            }
        }
        return false;
    }
    @Override public void clear() {
        clearOnAdd = false;
        for (Row row : rows) row.items.removeIf(data -> !data.retained());
        rows.removeIf(row -> row.items.isEmpty());
    }
    @Override public void clearAll() {
        clearOnAdd = false;
        rows.clear();
        actions.clear();
    }
    @Override public Object addAction(Runnable action) {
        Object token = new Object();
        actions.put(token, Objects.requireNonNull(action));
        return token;
    }
    @Override public boolean removeAction(Object token) { return actions.remove(token) != null; }
    @Override public boolean update() {
        for (Runnable action : new ArrayList<>(actions.values())) action.run();
        List<Entry> entries = new ArrayList<>();
        for (Row row : rows) {
            if (row.caption != null) entries.add(new Entry(row.caption, ""));
            for (Data data : row.items) entries.add(new Entry(data.caption, data.render()));
        }
        updates.add(new Update(entries, log.entries()));
        clearOnAdd = autoClear;
        return true;
    }
    @Override public boolean isAutoClear() { return autoClear; }
    @Override public void setAutoClear(boolean value) { autoClear = value; }
    @Override public int getMsTransmissionInterval() { return interval; }
    @Override public void setMsTransmissionInterval(int value) {
        if (value < 0) throw new IllegalArgumentException("Negative transmission interval");
        interval = value;
    }
    @Override public String getItemSeparator() { return itemSeparator; }
    @Override public void setItemSeparator(String value) { itemSeparator = Objects.requireNonNull(value); }
    @Override public String getCaptionValueSeparator() { return captionSeparator; }
    @Override public void setCaptionValueSeparator(String value) {
        captionSeparator = Objects.requireNonNull(value);
    }
    @Override public void setDisplayFormat(DisplayFormat value) {
        displayFormat = Objects.requireNonNull(value);
    }
    public DisplayFormat displayFormat() { return displayFormat; }
    @Override public void setNumDecimalPlaces(int min, int max) {
        throw new UnsupportedOperationException("Use explicit telemetry format strings");
    }
    @Override public void speak(String text) {
        throw new UnsupportedOperationException("Desktop telemetry does not implement speech");
    }
    @Override public void speak(String text, String language, String country) { speak(text); }
    @Override public Log log() { return log; }

    private Row newRow(String caption) {
        beforeAddition();
        Row row = new Row(caption);
        rows.add(row);
        return row;
    }

    private void beforeAddition() {
        if (clearOnAdd) clear();
    }

    private final class Row implements InvocationHandler {
        final String caption;
        final List<Data> items = new ArrayList<>();
        final Line line = proxy(Line.class, this);

        Row(String caption) { this.caption = caption; }
        Item add(String caption, Object value, String format, boolean functional) {
            beforeAddition();
            if (!rows.contains(this)) throw new IllegalStateException("Telemetry line was removed");
            Data data = new Data(this, caption, value, format, functional);
            items.add(data);
            return data.item;
        }
        @Override public Object invoke(Object proxy, Method method, Object[] args) {
            if (method.getDeclaringClass() == Object.class) return objectMethod(proxy, method, args);
            if (method.getName().equals("addData")) return addArguments(this, method, args);
            throw unsupported(method);
        }
    }

    private final class Data implements InvocationHandler {
        final Row row;
        final Item item = proxy(Item.class, this);
        String caption;
        Object value;
        String format;
        boolean functional;
        Boolean retain;

        Data(Row row, String caption, Object value, String format, boolean functional) {
            this.row = row;
            this.caption = Objects.requireNonNull(caption);
            this.value = value;
            this.format = format;
            this.functional = functional;
        }
        boolean retained() { return retain == null ? functional : retain; }
        String render() {
            Object result = functional ? ((Func<?>) value).value() : value;
            return format == null ? (result == null ? "" : String.valueOf(result))
                    : String.format(Locale.ROOT, format, result);
        }
        @Override public Object invoke(Object proxy, Method method, Object[] args) {
            if (method.getDeclaringClass() == Object.class) return objectMethod(proxy, method, args);
            switch (method.getName()) {
                case "getCaption": return caption;
                case "setCaption": caption = Objects.requireNonNull((String) args[0]); return item;
                case "isRetained": return retained();
                case "setRetained": retain = (Boolean) args[0]; return item;
                case "setValue":
                    Object argument = args[args.length - 1];
                    functional = method.getParameterTypes()[args.length - 1] == Func.class;
                    if (functional) Objects.requireNonNull(argument, "valueProducer");
                    format = args.length == 2 && functional ? (String) args[0] : null;
                    value = args.length == 2 && !functional
                            ? String.format(Locale.ROOT, (String) args[0], (Object[]) args[1])
                            : argument;
                    return item;
                case "addData":
                    beforeAddition();
                    Row target = row;
                    if (!rows.contains(target) && target.caption == null) {
                        target = newRow(null);
                        rows.remove(target);
                        rows.add(0, target);
                    }
                    Item added = addArguments(target, method, args);
                    Data data = target.items.remove(target.items.size() - 1);
                    target.items.add(target.items.indexOf(this) + 1, data);
                    return added;
                default: throw unsupported(method);
            }
        }
    }

    private Item addArguments(Row row, Method method, Object[] args) {
        String caption = (String) args[0];
        Object argument = args[args.length - 1];
        boolean functional = method.getParameterTypes()[args.length - 1] == Func.class;
        if (functional) Objects.requireNonNull(argument, "valueProducer");
        if (args.length == 3 && !functional) {
            return row.add(caption, String.format(Locale.ROOT, (String) args[1],
                    (Object[]) args[2]), null, false);
        }
        return row.add(caption, argument, args.length == 3 ? (String) args[1] : null, functional);
    }

    private static <T> T proxy(Class<T> type, InvocationHandler handler) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, handler));
    }
    private static Object objectMethod(Object proxy, Method method, Object[] args) {
        switch (method.getName()) {
            case "equals": return proxy == args[0];
            case "hashCode": return System.identityHashCode(proxy);
            case "toString": return "RecordingTelemetry." + proxy.getClass().getInterfaces()[0].getSimpleName();
            default: throw unsupported(method);
        }
    }
    private static UnsupportedOperationException unsupported(Method method) {
        return new UnsupportedOperationException("Unsupported telemetry method: " + method);
    }

    private static final class RecordingLog implements Log {
        final List<String> messages = new ArrayList<>();
        int capacity = 9;
        DisplayOrder order = DisplayOrder.OLDEST_FIRST;
        @Override public int getCapacity() { return capacity; }
        @Override public void setCapacity(int value) {
            if (value < 0) throw new IllegalArgumentException("Negative log capacity");
            capacity = value;
            trim();
        }
        @Override public DisplayOrder getDisplayOrder() { return order; }
        @Override public void setDisplayOrder(DisplayOrder value) { order = Objects.requireNonNull(value); }
        @Override public void add(String entry) { messages.add(Objects.requireNonNull(entry)); trim(); }
        @Override public void add(String format, Object... args) {
            add(String.format(Locale.ROOT, format, args));
        }
        @Override public void clear() { messages.clear(); }
        void trim() { while (messages.size() > capacity) messages.remove(0); }
        List<String> entries() {
            List<String> result = new ArrayList<>(messages);
            if (order == DisplayOrder.NEWEST_FIRST) Collections.reverse(result);
            return result;
        }
    }
}
