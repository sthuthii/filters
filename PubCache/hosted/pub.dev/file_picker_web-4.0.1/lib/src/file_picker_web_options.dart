import 'package:file_picker_platform_interface/file_picker_platform_interface.dart';

/// Configuration options specific to the Web platform.
final class FilePickerWebOptions extends WebOptions {
  /// Whether to read each picked file into memory at pick time.
  ///
  /// Files larger than 2 GB are never preloaded, since browsers cannot hold
  /// them in a single buffer. Their content can be read on demand through
  /// `PlatformFile.readAsBytes()` and `readAsByteStream()` instead.
  final bool withData;

  /// Whether to create a read stream for each picked file.
  final bool withReadStream;

  /// Whether to read multiple files one at a time instead of concurrently.
  ///
  /// The result always preserves the original selection order, regardless
  /// of this setting.
  final bool readSequential;

  /// Whether to cancel upload when window loses focus.
  final bool cancelUploadOnWindowBlur;

  /// Creates an instance of [FilePickerWebOptions].
  const FilePickerWebOptions({
    this.withData = true,
    this.withReadStream = false,
    this.readSequential = false,
    this.cancelUploadOnWindowBlur = true,
  });
}
