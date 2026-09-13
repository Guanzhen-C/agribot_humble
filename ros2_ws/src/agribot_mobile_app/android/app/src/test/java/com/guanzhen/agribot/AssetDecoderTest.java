package com.guanzhen.agribot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Base64;
import java.util.zip.GZIPOutputStream;

public final class AssetDecoderTest {
    private static final String CONTENT = "window.unityFramework = { ready: true };";
    private static final String BROTLI_FIXTURE =
        "ixOAd2luZG93LnVuaXR5RnJhbWV3b3JrID0geyByZWFkeTogdHJ1ZSB9OwM=";

    @Rule
    public final TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void decodesBrotliToARegularFile() throws Exception {
        File source = temporaryFolder.newFile("framework.js.br");
        File destination = new File(temporaryFolder.getRoot(), "framework.js");
        Files.write(source.toPath(), Base64.getDecoder().decode(BROTLI_FIXTURE));

        AssetDecoder.Result result = AssetDecoder.decode(
            source,
            destination,
            "br",
            1024L,
            0L,
            () -> false
        );

        assertEquals(
            CONTENT,
            new String(Files.readAllBytes(destination.toPath()), StandardCharsets.UTF_8)
        );
        assertEquals(source.length(), result.sourceSize);
        assertEquals(CONTENT.getBytes(StandardCharsets.UTF_8).length, result.decodedSize);
        assertEquals(64, result.sourceSha256.length());
        assertEquals(64, result.decodedSha256.length());
    }

    @Test
    public void decodesGzipToARegularFile() throws Exception {
        File source = temporaryFolder.newFile("framework.js.gz");
        try (GZIPOutputStream stream = new GZIPOutputStream(new FileOutputStream(source))) {
            stream.write(CONTENT.getBytes(StandardCharsets.UTF_8));
        }
        File destination = new File(temporaryFolder.getRoot(), "framework.js");

        AssetDecoder.Result result = AssetDecoder.decode(
            source,
            destination,
            "gzip",
            1024L,
            0L,
            () -> false
        );

        assertEquals(
            CONTENT,
            new String(Files.readAllBytes(destination.toPath()), StandardCharsets.UTF_8)
        );
        assertEquals(CONTENT.getBytes(StandardCharsets.UTF_8).length, result.decodedSize);
    }

    @Test(expected = IOException.class)
    public void removesPartialOutputWhenDecodedSizeExceedsTheLimit() throws Exception {
        File source = temporaryFolder.newFile("framework.js.br");
        File destination = new File(temporaryFolder.getRoot(), "framework.js");
        Files.write(source.toPath(), Base64.getDecoder().decode(BROTLI_FIXTURE));

        try {
            AssetDecoder.decode(source, destination, "br", 8L, 0L, () -> false);
        } finally {
            assertFalse(destination.exists());
        }
    }
}
