package build.utils.tuning.swerveencoders;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.util.DefaultIndenter;
import com.fasterxml.jackson.core.util.DefaultPrettyPrinter;

public final class ConfigPrettyPrinter extends DefaultPrettyPrinter {
    private static final long serialVersionUID = 1L;

    public ConfigPrettyPrinter() {
        this._objectFieldValueSeparatorWithSpaces = ": ";
        this.indentArraysWith(DefaultIndenter.SYSTEM_LINEFEED_INSTANCE);
        this.indentObjectsWith(new DefaultIndenter("    ", DefaultIndenter.SYS_LF)); // 4 spaces
    }

    @Override
    public DefaultPrettyPrinter createInstance() {
        return new ConfigPrettyPrinter();
    }

    @Override
    public void writeStartArray(JsonGenerator g) throws java.io.IOException {
        if (!_arrayIndenter.isInline()) {
            ++_nesting;
        }
        g.writeRaw('[');
    }

    @Override
    public void writeEndArray(JsonGenerator g, int numValues) throws java.io.IOException {
        if (!_arrayIndenter.isInline()) {
            --_nesting;
        }
        if (numValues > 0) {
            _arrayIndenter.writeIndentation(g, _nesting);
        }

        g.writeRaw(']');
    }
}
