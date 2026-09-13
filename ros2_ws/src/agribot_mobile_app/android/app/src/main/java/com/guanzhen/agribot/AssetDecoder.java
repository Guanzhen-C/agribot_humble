package com.guanzhen.agribot;

import org.brotli.dec.BrotliInputStream;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.zip.GZIPInputStream;

/** Streams a compressed asset into a verified identity file without using the Java heap. */
final class AssetDecoder {
    private static final int BUFFER_SIZE = 128 * 1024;
    private static final long SPACE_CHECK_INTERVAL_BYTES = 8L * 1024L * 1024L;

    private AssetDecoder() {
    }

    static Result decode(
        File source,
        File destination,
        String encoding,
        long maximumOutputBytes,
        long minimumFreeBytes,
        BooleanSupplier cancelled
    ) throws IOException {
        MessageDigest sourceDigest = sha256Digest();
        MessageDigest decodedDigest = sha256Digest();
        long decodedSize = 0L;
        long nextSpaceCheck = 0L;

        try (
            FileInputStream rawStream = new FileInputStream(source);
            DigestInputStream digestStream = new DigestInputStream(rawStream, sourceDigest);
            InputStream decodedStream = decoder(digestStream, encoding);
            FileOutputStream outputStream = new FileOutputStream(destination)
        ) {
            byte[] buffer = new byte[BUFFER_SIZE];
            int count;
            while ((count = decodedStream.read(buffer)) != -1) {
                if (Thread.currentThread().isInterrupted() || cancelled.getAsBoolean()) {
                    throw new IOException("三维配置资源准备已取消");
                }
                decodedSize += count;
                if (decodedSize > maximumOutputBytes) {
                    throw new IOException("三维配置资源解压后大小超限");
                }
                if (decodedSize >= nextSpaceCheck) {
                    long usableSpace = destination.getParentFile().getUsableSpace();
                    if (usableSpace > 0L && usableSpace < minimumFreeBytes) {
                        throw new IOException("应用私有存储空间不足，无法解压三维配置资源");
                    }
                    nextSpaceCheck = decodedSize + SPACE_CHECK_INTERVAL_BYTES;
                }
                outputStream.write(buffer, 0, count);
                decodedDigest.update(buffer, 0, count);
            }
            outputStream.getFD().sync();
        } catch (IOException | RuntimeException error) {
            destination.delete();
            throw error;
        }

        return new Result(
            source.length(),
            decodedSize,
            hex(sourceDigest.digest()),
            hex(decodedDigest.digest())
        );
    }

    private static InputStream decoder(InputStream stream, String encoding) throws IOException {
        if ("br".equals(encoding)) {
            return new BrotliInputStream(stream);
        }
        if ("gzip".equals(encoding)) {
            return new GZIPInputStream(stream, BUFFER_SIZE);
        }
        throw new IOException("三维配置资源使用了不支持的压缩格式");
    }

    private static MessageDigest sha256Digest() throws IOException {
        try {
            return MessageDigest.getInstance("SHA-256");
        } catch (NoSuchAlgorithmException error) {
            throw new IOException("系统不支持SHA-256", error);
        }
    }

    private static String hex(byte[] value) {
        StringBuilder result = new StringBuilder(value.length * 2);
        for (byte item : value) {
            result.append(String.format(Locale.ROOT, "%02x", item & 0xff));
        }
        return result.toString();
    }

    static final class Result {
        final long sourceSize;
        final long decodedSize;
        final String sourceSha256;
        final String decodedSha256;

        Result(
            long sourceSize,
            long decodedSize,
            String sourceSha256,
            String decodedSha256
        ) {
            this.sourceSize = sourceSize;
            this.decodedSize = decodedSize;
            this.sourceSha256 = sourceSha256;
            this.decodedSha256 = decodedSha256;
        }
    }
}
