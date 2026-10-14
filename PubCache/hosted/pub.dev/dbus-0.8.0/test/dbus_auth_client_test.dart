import 'dart:async';

import 'package:dbus/dbus.dart';
import 'package:test/test.dart';

void main() {
  group('DBusAuthClient', () {
    test('rejects anonymous authentication by default', () async {
      var client = DBusAuthClient(requestUnixFd: false);
      var requests = StreamIterator(client.requests);
      addTearDown(requests.cancel);

      expect(await requests.moveNext(), isTrue);
      expect(requests.current, '\x00AUTH');

      client.processResponse('REJECTED ANONYMOUS');

      expect(await requests.moveNext(), isTrue);
      expect(requests.current, 'ERROR No supported mechanism');
      await client.done;
      expect(client.isAuthenticated, isFalse);
    });

    test('falls back to anonymous authentication when enabled', () async {
      var client = DBusAuthClient(
        requestUnixFd: false,
        allowAnonymous: true,
        uid: '0',
      );
      var requests = StreamIterator(client.requests);
      addTearDown(requests.cancel);

      expect(await requests.moveNext(), isTrue);
      expect(requests.current, '\x00AUTH');

      client.processResponse('REJECTED EXTERNAL ANONYMOUS');

      expect(await requests.moveNext(), isTrue);
      expect(requests.current, 'AUTH EXTERNAL 30');

      client.processResponse('REJECTED EXTERNAL ANONYMOUS');

      expect(await requests.moveNext(), isTrue);
      expect(requests.current, 'AUTH ANONYMOUS');

      client.processResponse('OK 00112233445566778899aabbccddeeff');

      expect(await requests.moveNext(), isTrue);
      expect(requests.current, 'BEGIN');
      await client.done;
      expect(client.isAuthenticated, isTrue);
    });
  });
}
