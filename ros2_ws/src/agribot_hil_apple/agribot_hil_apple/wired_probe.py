"""ROS round-trip probe with a session nonce; it never sends chassis commands."""
import argparse
import json
from pathlib import Path
import platform
import statistics
import time
import uuid


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('server', 'client'))
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--payload-bytes', type=int, default=16384)
    parser.add_argument('--timeout', type=float, default=25)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.count <= 10000 or not 0 <= args.payload_bytes <= 65536 or args.timeout <= 0:
        parser.error('invalid probe limits')
    rclpy.init(args=[])
    node = rclpy.create_node('hil_wired_' + args.mode)
    qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
    outgoing = '/hil/link_probe/' + ('response' if args.mode == 'server' else 'request')
    incoming = '/hil/link_probe/' + ('request' if args.mode == 'server' else 'response')
    publisher = node.create_publisher(String, outgoing, qos)
    session = uuid.uuid4().hex
    sent, replies = {}, {}
    peer = {}

    def receive(message):
        try:
            body = json.loads(message.data)
            if not isinstance(body.get('session'), str) or type(body.get('sequence')) is not int:
                return
            if args.mode == 'server':
                body['responder'] = {'hostname': platform.node(), 'architecture': platform.machine()}
                publisher.publish(String(data=json.dumps(body, separators=(',', ':'))))
            elif body['session'] == session and body['sequence'] in sent:
                sequence = body['sequence']
                if body.get('payload') != 'x' * args.payload_bytes or sequence in replies:
                    return
                replies[sequence] = (time.monotonic_ns() - sent[sequence]) / 1e6
                peer.update(body.get('responder', {}))
        except (ValueError, KeyError, TypeError):
            return

    subscription = node.create_subscription(String, incoming, receive, qos)
    try:
        if args.mode == 'server':
            deadline = time.monotonic() + args.timeout
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            return
        deadline = time.monotonic() + args.timeout
        discovery_end = min(deadline, time.monotonic() + 8)
        while publisher.get_subscription_count() == 0 and time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        next_send = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline and len(replies) < args.count:
            now = time.monotonic()
            if len(sent) < args.count and now >= next_send:
                sequence = len(sent)
                sent[sequence] = time.monotonic_ns()
                publisher.publish(String(data=json.dumps({'session': session, 'sequence': sequence,
                    'payload': 'x' * args.payload_bytes}, separators=(',', ':'))))
                next_send = now + 0.05
            rclpy.spin_once(node, timeout_sec=0.02)
        values = sorted(replies.values())
        report = {'ok': len(replies) == args.count and peer.get('architecture') == 'aarch64',
                  'session': session, 'sent': len(sent), 'received': len(replies),
                  'payload_bytes': args.payload_bytes, 'peer': peer,
                  'rtt_median_ms': statistics.median(values) if values else None,
                  'rtt_p95_ms': values[min(len(values)-1, int(len(values)*0.95))] if values else None,
                  'clock_basis': 'PC monotonic request-to-echo RTT, not one-way sensor delay'}
        text = json.dumps(report, indent=2)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + '\n')
        print(text, flush=True)
        if not report['ok']:
            raise SystemExit(1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
