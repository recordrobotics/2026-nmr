package build.utils;

import com.fasterxml.jackson.core.PrettyPrinter;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

public final class FileUtils {

    private static final ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

    private FileUtils() {}

    public static File getLaunchDirectory() {
        // workaround for
        // https://www.chiefdelphi.com/t/filesystem-getdeploydirectory-returning-wrong-location-how-to-fix/427292
        String path = System.getProperty("user.dir")
                .replace(File.separator + "build" + File.separator + "jni" + File.separator + "release", "");
        return new File(path).getAbsoluteFile();
    }

    public static JsonNode readJson(File file) throws FileNotFoundException {
        if (!file.exists()) throw new FileNotFoundException("File not found: " + file.getAbsolutePath());

        JsonNode obj;
        try {
            obj = mapper.readTree(file);
        } catch (IOException e) {
            throw new InvalidFileFormatException("Error reading JSON file: " + file.getAbsolutePath(), e);
        }

        return obj;
    }

    public static void writeJson(File file, JsonNode obj, PrettyPrinter feature) {
        try (FileWriter writer = new FileWriter(file, StandardCharsets.UTF_8)) {
            mapper.writer(feature).writeValue(writer, obj);
        } catch (IOException e) {
            throw new InvalidFileFormatException("Error writing JSON file: " + file.getAbsolutePath(), e);
        }
    }

    public static class InvalidFileFormatException extends RuntimeException {
        public InvalidFileFormatException(String message) {
            super(message);
        }

        public InvalidFileFormatException(String message, Throwable cause) {
            super(message, cause);
        }
    }
}
