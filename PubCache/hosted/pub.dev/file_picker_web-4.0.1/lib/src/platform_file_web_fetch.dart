import 'dart:js_interop';
import 'dart:typed_data';

@JS('fetch')
external JSPromise<JSObject> _fetchJs(JSString url);

/// Interop extension type representing a Web `Response` JS object.
extension type _Response(JSObject _) implements JSObject {
  external JSPromise<JSArrayBuffer> arrayBuffer();
  external JSObject? get body;
}

/// Interop extension type representing a Web `ReadableStream` JS object.
extension type _ReadableStream(JSObject _) implements JSObject {
  external JSObject getReader();
}

/// Interop extension type representing a Web `ReadableStreamDefaultReader` JS object.
extension type _Reader(JSObject _) implements JSObject {
  external JSPromise<JSObject> read();
}

/// Interop extension type representing a Web `ReadableStreamReadResult` JS object.
extension type _ReadResult(JSObject _) implements JSObject {
  external bool get done;
  external JSUint8Array? get value;
}

/// Fetches the bytes of a web-only path (`blob:` or `data:` URL).
///
/// Returns the full file bytes (`Uint8List`) using `fetch(...).arrayBuffer()`
/// for `blob:` URLs, or parses `data:` URIs. Returns `null` if [path] is not a
/// web URL, so the caller can fall back to another source. Throws if the
/// content of a web URL cannot be read.
Future<Uint8List?> fetchBytesFromWebPath(String path) async {
  if (!_isWebPath(path)) return null;

  if (path.startsWith('data:')) {
    final uriData = Uri.parse(path).data;

    if (uriData == null) {
      return null;
    }

    return uriData.contentAsBytes();
  }

  final response = _Response(await _fetchJs(path.toJS).toDart);
  final buffer = await response.arrayBuffer().toDart;
  return buffer.toDart.asUint8List();
}

bool _isWebPath(String path) =>
    path.startsWith('blob:') || path.startsWith('data:');

/// Attempts to create a streaming `Stream<Uint8List>` from a web-only path
/// (`blob:` or `data:` URL).
///
/// Returns `null` if [path] is not a web URL, so the caller can fall back to
/// another source. Read failures are emitted as errors on the stream.
Stream<Uint8List>? fetchStreamFromWebPath(String path) {
  if (!_isWebPath(path)) return null;

  return _streamFromWebPath(path);
}

/// Reads a `blob:` or `data:` URL and emits its bytes as a stream.
///
/// Uses `Response.body` (`ReadableStream`) when available; otherwise falls
/// back to a single in-memory `arrayBuffer()` chunk.
Stream<Uint8List> _streamFromWebPath(String path) async* {
  if (path.startsWith('data:')) {
    final uriData = Uri.parse(path).data;

    if (uriData == null) {
      throw FormatException('Invalid data: URL', path);
    }

    yield uriData.contentAsBytes();
    return;
  }

  final response = _Response(await _fetchJs(path.toJS).toDart);
  final body = response.body;

  // If there's no streaming body, fallback to arrayBuffer()
  if (body == null) {
    final buffer = await response.arrayBuffer().toDart;
    yield buffer.toDart.asUint8List();
    return;
  }

  final readable = _ReadableStream(body);
  final reader = _Reader(readable.getReader());

  while (true) {
    final result = _ReadResult(await reader.read().toDart);
    if (result.done) break;

    final arr = result.value;
    if (arr == null) break;

    yield arr.toDart;
  }
}
