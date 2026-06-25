package build.utils.tuning;

import java.util.Locale;

public final class Main {

    private Main() {}

    @SuppressWarnings("java:S1301")
    public static void main(String... args) {
        if (args.length == 0) {
            throw new IllegalArgumentException("Please provide a tuning module to run (e.g., swerveEncoders)");
        }

        switch (args[0].toLowerCase(Locale.ENGLISH)) {
            case "swerveencoders" -> build.utils.tuning.swerveencoders.Main.main(trimFirstArg(args));
            default -> throw new IllegalArgumentException("Unknown tuning module: " + args[0]);
        }
    }

    private static String[] trimFirstArg(String[] args) {
        if (args.length <= 1) {
            return new String[0];
        }
        String[] trimmed = new String[args.length - 1];
        System.arraycopy(args, 1, trimmed, 0, args.length - 1);
        return trimmed;
    }
}
