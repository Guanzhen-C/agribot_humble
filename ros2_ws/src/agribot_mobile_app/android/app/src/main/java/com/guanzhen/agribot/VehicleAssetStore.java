package com.guanzhen.agribot;

import android.content.Context;
import android.content.SharedPreferences;
import android.content.res.AssetManager;
import android.net.Uri;
import android.webkit.MimeTypeMap;
import android.webkit.WebResourceResponse;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;

/** Stores a verified Unity WebGL build in app-private storage. */
public final class VehicleAssetStore implements AutoCloseable {
    public interface StateListener {
        void onStateChanged(String stateJson);
    }

    public static final String APP_ASSET_ORIGIN = "https://appassets.androidplatform.net";
    public static final String BUNDLED_UI_URL = APP_ASSET_ORIGIN + "/assets/web/index.html";

    private static final Object PACKAGE_IO_LOCK = new Object();
    private static final String APP_ASSET_HOST = "appassets.androidplatform.net";
    private static final String BUNDLED_PREFIX = "/assets/web/";
    private static final String VEHICLE_PREFIX = "/vehicle-webgl-identity-v1/";
    private static final String MANIFEST_PATH = "/api/v1/vehicle-config/manifest";
    private static final String GATEWAY_ASSET_PREFIX = "/vehicle-webgl/";
    private static final String CACHE_DIRECTORY = "vehicle-assets";
    private static final String VERSIONS_DIRECTORY = "versions";
    private static final String STAGING_DIRECTORY = "staging";
    private static final String STORED_MANIFEST = ".manifest.json";
    private static final String IDENTITY_DIRECTORY = "_identity";
    private static final String IDENTITY_STAGING_PREFIX = "_identity-staging-";
    private static final String IDENTITY_MANIFEST = "manifest.json";
    private static final int IDENTITY_SCHEMA_VERSION = 1;
    private static final String PREF_ACTIVE_VERSION = "vehicle_assets.active_version";
    private static final String PREF_ACTIVE_MANIFEST = "vehicle_assets.active_manifest";
    private static final int CONNECT_TIMEOUT_MS = 5000;
    private static final int READ_TIMEOUT_MS = 15000;
    private static final int BUFFER_SIZE = 128 * 1024;
    private static final int MAX_MANIFEST_BYTES = 2 * 1024 * 1024;
    private static final int MAX_FILE_COUNT = 4096;
    private static final long MAX_TOTAL_BYTES = 4L * 1024L * 1024L * 1024L;
    private static final long MAX_DECODED_FILE_BYTES = Integer.MAX_VALUE;
    private static final long MAX_DECODED_TOTAL_BYTES = 4L * 1024L * 1024L * 1024L;
    private static final long MIN_FREE_AFTER_DECODE_BYTES = 64L * 1024L * 1024L;
    private static final long PROGRESS_INTERVAL_MS = 250L;

    private final Object lock = new Object();
    private final Context context;
    private final SharedPreferences preferences;
    private final StateListener listener;
    private final ExecutorService downloadExecutor = Executors.newSingleThreadExecutor();
    private final File versionsRoot;
    private final File stagingRoot;

    private String activeVersion;
    private JSONObject activeManifest;
    private String stateJson;
    private boolean activePrepared;
    private boolean ensureInFlight;
    private boolean closed;

    public VehicleAssetStore(
        Context context,
        SharedPreferences preferences,
        StateListener listener
    ) {
        this.context = context.getApplicationContext();
        this.preferences = preferences;
        this.listener = listener;
        File cacheRoot = new File(this.context.getFilesDir(), CACHE_DIRECTORY);
        versionsRoot = new File(cacheRoot, VERSIONS_DIRECTORY);
        stagingRoot = new File(cacheRoot, STAGING_DIRECTORY);
        loadActivePackage();
        downloadExecutor.execute(() -> {
            synchronized (PACKAGE_IO_LOCK) {
                if (!isClosed()) {
                    cleanupAbandonedStaging();
                }
            }
        });
    }

    public String getStateJson() {
        synchronized (lock) {
            return stateJson;
        }
    }

    public void ensureAssets(String gatewayUrl) {
        boolean prepareOnly;
        synchronized (lock) {
            if (closed || ensureInFlight) {
                return;
            }
            ensureInFlight = true;
            prepareOnly = activeManifest != null && !activePrepared;
            if (prepareOnly) {
                setStateLocked(status("preparing", 0.0, "正在准备App本地三维资源"));
            } else if (activeManifest == null) {
                setStateLocked(status("downloading", 0.0, "正在读取三维配置资源清单"));
            }
            try {
                downloadExecutor.execute(
                    prepareOnly
                        ? () -> prepareActivePackage(gatewayUrl)
                        : () -> refreshFromGateway(gatewayUrl)
                );
            } catch (RejectedExecutionException error) {
                ensureInFlight = false;
                if (!closed) {
                    setStateLocked(status("error", 0.0, "无法启动三维配置资源任务"));
                }
                return;
            }
        }
        notifyState();
    }

    public boolean handles(Uri uri) {
        return isAppAssetUrl(uri);
    }

    public WebResourceResponse intercept(Uri uri) {
        if (!handles(uri)) {
            return null;
        }
        try {
            String path = uri.getPath();
            if (path == null) {
                return errorResponse(404, "Not Found");
            }
            if (path.startsWith(BUNDLED_PREFIX)) {
                return bundledResponse(path.substring(BUNDLED_PREFIX.length()));
            }
            if (path.startsWith(VEHICLE_PREFIX)) {
                return cachedResponse(path.substring(VEHICLE_PREFIX.length()));
            }
            return errorResponse(404, "Not Found");
        } catch (IOException | IllegalArgumentException error) {
            return errorResponse(404, "Not Found");
        }
    }

    public static boolean isBundledUiUrl(String url) {
        if (url == null) {
            return false;
        }
        Uri uri = Uri.parse(url);
        return "https".equalsIgnoreCase(uri.getScheme())
            && APP_ASSET_HOST.equalsIgnoreCase(uri.getHost())
            && uri.getPath() != null
            && uri.getPath().startsWith(BUNDLED_PREFIX);
    }

    public static boolean isAppAssetUrl(Uri uri) {
        return uri != null
            && "https".equalsIgnoreCase(uri.getScheme())
            && APP_ASSET_HOST.equalsIgnoreCase(uri.getHost());
    }

    @Override
    public void close() {
        synchronized (lock) {
            closed = true;
        }
        downloadExecutor.shutdownNow();
    }

    private void loadActivePackage() {
        String storedVersion = preferences.getString(PREF_ACTIVE_VERSION, "");
        String storedManifest = preferences.getString(PREF_ACTIVE_MANIFEST, "");
        if (!storedVersion.isEmpty() && !storedManifest.isEmpty()) {
            try {
                JSONObject manifest = validateManifest(new JSONObject(storedManifest));
                if (!storedVersion.equals(manifest.getString("version"))) {
                    throw new IOException("缓存版本和清单不一致");
                }
                File packageRoot = versionDirectory(storedVersion);
                validateCachedPackage(packageRoot, manifest, false);
                activeVersion = storedVersion;
                activeManifest = manifest;
                activePrepared = identityPackageReady(packageRoot, manifest);
                if (activePrepared) {
                    stateJson = readyState(manifest, storedVersion, null).toString();
                    return;
                }
                stateJson = status(
                    "preparing",
                    0.0,
                    "正在将已下载的三维资源转换为App可直接读取的格式"
                ).toString();
                return;
            } catch (IOException | JSONException | IllegalArgumentException ignored) {
                preferences.edit()
                    .remove(PREF_ACTIVE_VERSION)
                    .remove(PREF_ACTIVE_MANIFEST)
                    .apply();
            }
        }
        activePrepared = false;
        stateJson = status("idle", 0.0, "三维配置资源尚未缓存").toString();
    }

    private void refreshFromGateway(String gatewayUrl) {
        boolean hadUsableCache;
        boolean downloadAnnounced = false;
        synchronized (lock) {
            hadUsableCache = activeManifest != null && activePrepared;
        }
        try {
            JSONObject manifest = fetchManifest(gatewayUrl);
            String version = manifest.getString("version");
            synchronized (lock) {
                if (closed) {
                    return;
                }
                if (activeManifest != null && activePrepared && version.equals(activeVersion)) {
                    setStateLocked(readyState(activeManifest, activeVersion, null));
                    return;
                }
                setStateLocked(status("downloading", 0.0, "正在下载三维配置资源"));
            }
            downloadAnnounced = true;
            notifyState();
            downloadAndActivatePackage(gatewayUrl, manifest, version);
            notifyState();
        } catch (Exception error) {
            boolean shouldNotify;
            synchronized (lock) {
                if (closed) {
                    return;
                }
                if (activeManifest != null && hadUsableCache) {
                    setStateLocked(readyState(
                        activeManifest,
                        activeVersion,
                        "无法检查更新，继续使用已校验的本地资源"
                    ));
                    shouldNotify = downloadAnnounced;
                } else {
                    String message = error.getMessage();
                    if (message == null || message.trim().isEmpty()) {
                        message = "三维配置资源下载失败";
                    }
                    setStateLocked(status("error", 0.0, message));
                    shouldNotify = true;
                }
            }
            if (shouldNotify) {
                notifyState();
            }
        } finally {
            synchronized (lock) {
                ensureInFlight = false;
            }
        }
    }

    private void prepareActivePackage(String gatewayUrl) {
        String version;
        JSONObject manifest;
        boolean handedOffToDownload = false;
        boolean sourceInvalid = false;
        synchronized (lock) {
            version = activeVersion;
            manifest = activeManifest;
        }
        try {
            if (version == null || manifest == null) {
                throw new IOException("三维配置缓存状态无效");
            }
            File packageRoot = versionDirectory(version);
            synchronized (PACKAGE_IO_LOCK) {
                if (isClosed()) {
                    return;
                }
                try {
                    validateCachedPackage(packageRoot, manifest, true);
                } catch (IOException | JSONException invalidSource) {
                    invalidateActivePackage(version, packageRoot);
                    sourceInvalid = true;
                }
                if (!sourceInvalid) {
                    materializeIdentityPackage(packageRoot, manifest);
                }
            }
            if (sourceInvalid) {
                if (isClosed()) {
                    return;
                }
                synchronized (lock) {
                    setStateLocked(status(
                        "downloading",
                        0.0,
                        "本地三维资源校验失败，正在重新下载"
                    ));
                }
                notifyState();
                handedOffToDownload = true;
                refreshFromGateway(gatewayUrl);
                return;
            }
            synchronized (lock) {
                if (closed) {
                    return;
                }
                activePrepared = true;
                setStateLocked(readyState(manifest, version, null));
            }
            notifyState();
            if (!isClosed()) {
                handedOffToDownload = true;
                refreshFromGateway(gatewayUrl);
            }
        } catch (IOException | JSONException | RuntimeException error) {
            synchronized (lock) {
                if (closed) {
                    return;
                }
                activePrepared = false;
                String message = error.getMessage();
                if (message == null || message.trim().isEmpty()) {
                    message = "三维配置资源准备失败";
                }
                setStateLocked(status("error", 0.0, message));
            }
            notifyState();
        } finally {
            if (!handedOffToDownload) {
                synchronized (lock) {
                    ensureInFlight = false;
                }
            }
        }
    }

    private void invalidateActivePackage(String version, File packageRoot) {
        synchronized (lock) {
            if (version.equals(activeVersion)) {
                activeVersion = null;
                activeManifest = null;
                activePrepared = false;
            }
        }
        preferences.edit()
            .remove(PREF_ACTIVE_VERSION)
            .remove(PREF_ACTIVE_MANIFEST)
            .apply();
        deleteRecursively(packageRoot);
    }

    private JSONObject fetchManifest(String gatewayUrl) throws IOException, JSONException {
        HttpURLConnection connection = null;
        try {
            connection = openConnection(new URL(trimTrailingSlash(gatewayUrl) + MANIFEST_PATH));
            int status = connection.getResponseCode();
            if (status != HttpURLConnection.HTTP_OK) {
                throw new IOException("三维配置资源清单请求失败: HTTP " + status);
            }
            byte[] payload;
            try (InputStream stream = connection.getInputStream()) {
                payload = readLimited(stream, MAX_MANIFEST_BYTES);
            }
            return validateManifest(new JSONObject(new String(payload, StandardCharsets.UTF_8)));
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    private JSONObject validateManifest(JSONObject manifest) throws IOException, JSONException {
        if (!manifest.optBoolean("available", false)) {
            throw new IOException(manifest.optString("message", "三维配置资源尚未部署"));
        }
        String version = manifest.getString("version");
        validateVersion(version);
        JSONArray files = manifest.getJSONArray("files");
        if (files.length() == 0 || files.length() > MAX_FILE_COUNT) {
            throw new IOException("三维配置资源文件数量异常");
        }

        Set<String> paths = new HashSet<>();
        long totalBytes = 0L;
        for (int index = 0; index < files.length(); index += 1) {
            JSONObject entry = files.getJSONObject(index);
            String path = entry.getString("path");
            validateManifestAssetPath(path);
            if (!paths.add(path)) {
                throw new IOException("三维配置资源清单包含重复路径");
            }
            long size = entry.getLong("size");
            if (size < 0L || size > MAX_TOTAL_BYTES) {
                throw new IOException("三维配置资源文件大小异常");
            }
            String checksum = entry.getString("sha256");
            if (!checksum.matches("(?i)[0-9a-f]{64}")) {
                throw new IOException("三维配置资源校验值无效");
            }
            if (Long.MAX_VALUE - totalBytes < size) {
                throw new IOException("三维配置资源总大小超限");
            }
            totalBytes += size;
            if (totalBytes > MAX_TOTAL_BYTES) {
                throw new IOException("三维配置资源总大小超限");
            }
        }
        if (!manifest.has("total_bytes") || manifest.getLong("total_bytes") != totalBytes) {
            throw new IOException("三维配置资源总大小与清单不一致");
        }

        JSONObject unity = manifest.getJSONObject("unity");
        for (String role : new String[]{"loader", "data", "framework", "code"}) {
            String path = unity.getString(role);
            validateManifestAssetPath(path);
            if (!paths.contains(path)) {
                throw new IOException("Unity资源清单缺少" + role);
            }
        }
        return manifest;
    }

    private void downloadAndActivatePackage(
        String gatewayUrl,
        JSONObject manifest,
        String version
    ) throws IOException, JSONException {
        synchronized (PACKAGE_IO_LOCK) {
            if (isClosed()) {
                throw new IOException("三维配置资源下载已取消");
            }
            File packageRoot = downloadPackageLocked(gatewayUrl, manifest);
            if (isClosed()) {
                throw new IOException("三维配置资源下载已取消");
            }
            activatePackageLocked(packageRoot, manifest, version);
        }
    }

    private File downloadPackageLocked(String gatewayUrl, JSONObject manifest)
        throws IOException, JSONException {
        ensureDirectory(versionsRoot);
        ensureDirectory(stagingRoot);
        String version = manifest.getString("version");
        File destination = versionDirectory(version);
        if (destination.exists() && !destination.isDirectory()) {
            if (!destination.delete()) {
                throw new IOException("无法清理损坏的三维配置缓存");
            }
        }
        if (destination.isDirectory()) {
            boolean cachedPackageValid = true;
            try {
                validateCachedPackage(destination, manifest, true);
            } catch (IOException invalidCache) {
                cachedPackageValid = false;
                deleteRecursively(destination);
            }
            if (cachedPackageValid) {
                materializeIdentityPackage(destination, manifest);
                return destination;
            }
            if (destination.exists()) {
                throw new IOException("无法清理损坏的三维配置缓存");
            }
        }

        File staging = new File(stagingRoot, version + "-" + UUID.randomUUID());
        ensureDirectory(staging);
        long totalBytes = manifest.getLong("total_bytes");
        if (stagingRoot.getUsableSpace() < totalBytes + (32L * 1024L * 1024L)) {
            deleteRecursively(staging);
            throw new IOException("应用私有存储空间不足");
        }

        Progress progress = new Progress(totalBytes);
        try {
            JSONArray files = manifest.getJSONArray("files");
            for (int index = 0; index < files.length(); index += 1) {
                if (Thread.currentThread().isInterrupted() || isClosed()) {
                    throw new IOException("三维配置资源下载已取消");
                }
                JSONObject entry = files.getJSONObject(index);
                downloadFile(gatewayUrl, staging, entry, progress);
            }
            writeManifest(staging, manifest);
            validateCachedPackage(staging, manifest, false);
            if (!staging.renameTo(destination)) {
                throw new IOException("无法原子切换三维配置资源版本");
            }
            materializeIdentityPackage(destination, manifest);
            return destination;
        } catch (IOException | JSONException error) {
            deleteRecursively(staging);
            throw error;
        }
    }

    private void downloadFile(
        String gatewayUrl,
        File staging,
        JSONObject entry,
        Progress progress
    ) throws IOException, JSONException {
        String relativePath = entry.getString("path");
        long expectedSize = entry.getLong("size");
        String expectedChecksum = entry.getString("sha256").toLowerCase(Locale.ROOT);
        File output = safeChild(staging, relativePath);
        File parent = output.getParentFile();
        if (parent == null) {
            throw new IOException("三维配置资源目标路径无效");
        }
        ensureDirectory(parent);

        HttpURLConnection connection = null;
        MessageDigest digest = sha256Digest();
        long written = 0L;
        try {
            URL url = new URL(
                trimTrailingSlash(gatewayUrl) + GATEWAY_ASSET_PREFIX + encodePath(relativePath)
            );
            connection = openConnection(url);
            connection.setRequestProperty("X-Agribot-Raw-Asset", "1");
            int status = connection.getResponseCode();
            if (status != HttpURLConnection.HTTP_OK) {
                throw new IOException("三维配置资源请求失败: HTTP " + status);
            }
            long responseSize = connection.getContentLengthLong();
            if (responseSize >= 0L && responseSize != expectedSize) {
                throw new IOException("三维配置资源响应大小不一致");
            }
            try (
                InputStream input = connection.getInputStream();
                FileOutputStream stream = new FileOutputStream(output)
            ) {
                byte[] buffer = new byte[BUFFER_SIZE];
                int count;
                while ((count = input.read(buffer)) != -1) {
                    if (Thread.currentThread().isInterrupted() || isClosed()) {
                        throw new IOException("三维配置资源下载已取消");
                    }
                    written += count;
                    if (written > expectedSize) {
                        throw new IOException("三维配置资源文件大小超出清单");
                    }
                    stream.write(buffer, 0, count);
                    digest.update(buffer, 0, count);
                    progress.add(count, relativePath);
                }
                stream.getFD().sync();
            }
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
        if (written != expectedSize) {
            throw new IOException("三维配置资源文件大小校验失败");
        }
        if (!hex(digest.digest()).equals(expectedChecksum)) {
            throw new IOException("三维配置资源SHA-256校验失败");
        }
    }

    private void activatePackageLocked(File packageRoot, JSONObject manifest, String version)
        throws IOException {
        String previousVersion;
        synchronized (lock) {
            previousVersion = activeVersion;
        }
        if (!preferences.edit()
            .putString(PREF_ACTIVE_VERSION, version)
            .putString(PREF_ACTIVE_MANIFEST, manifest.toString())
            .commit()) {
            throw new IOException("无法保存三维配置资源版本");
        }
        synchronized (lock) {
            if (closed) {
                return;
            }
            activeVersion = version;
            activeManifest = manifest;
            activePrepared = true;
            setStateLocked(readyState(manifest, version, null));
        }
        cleanupOldVersions(version, previousVersion, packageRoot);
    }

    private void validateCachedPackage(File root, JSONObject manifest, boolean verifyChecksum)
        throws IOException, JSONException {
        if (!root.isDirectory()) {
            throw new IOException("三维配置缓存目录不存在");
        }
        JSONArray files = manifest.getJSONArray("files");
        for (int index = 0; index < files.length(); index += 1) {
            JSONObject entry = files.getJSONObject(index);
            File file = safeChild(root, entry.getString("path"));
            if (!file.isFile() || file.length() != entry.getLong("size")) {
                throw new IOException("三维配置缓存不完整");
            }
            if (verifyChecksum) {
                String expected = entry.getString("sha256").toLowerCase(Locale.ROOT);
                if (!sha256(file).equals(expected)) {
                    throw new IOException("三维配置缓存校验失败");
                }
            }
        }
    }

    private boolean identityPackageReady(File packageRoot, JSONObject manifest) {
        try {
            validateIdentityPackage(packageRoot, manifest, false);
            return true;
        } catch (IOException | JSONException error) {
            return false;
        }
    }

    private void materializeIdentityPackage(File packageRoot, JSONObject manifest)
        throws IOException, JSONException {
        int compressedFileCount = compressedFileCount(manifest);
        if (compressedFileCount == 0 || identityPackageReady(packageRoot, manifest)) {
            return;
        }

        cleanupIdentityStaging(packageRoot);
        File identityRoot = new File(packageRoot, IDENTITY_DIRECTORY);
        if (identityRoot.exists()) {
            deleteRecursively(identityRoot);
        }
        File staging = new File(
            packageRoot,
            IDENTITY_STAGING_PREFIX + UUID.randomUUID()
        );
        ensureDirectory(staging);

        JSONObject identityManifest = new JSONObject();
        JSONArray identityFiles = new JSONArray();
        Set<String> decodedPaths = new HashSet<>();
        long decodedTotal = 0L;
        int compressedIndex = 0;
        try {
            JSONArray files = manifest.getJSONArray("files");
            for (int index = 0; index < files.length(); index += 1) {
                JSONObject entry = files.getJSONObject(index);
                String relativePath = entry.getString("path");
                ContentMetadata metadata = contentMetadata(relativePath);
                if (metadata.contentEncoding == null) {
                    continue;
                }
                compressedIndex += 1;
                publishPreparationProgress(
                    (double) (compressedIndex - 1) / (double) compressedFileCount,
                    relativePath
                );

                String decodedRelativePath = decodedRelativePath(
                    relativePath,
                    metadata.contentEncoding
                );
                if (!decodedPaths.add(decodedRelativePath)) {
                    throw new IOException("三维配置资源解压路径冲突");
                }
                File source = safeChild(packageRoot, relativePath);
                if (!source.isFile() || source.length() != entry.getLong("size")) {
                    throw new IOException("三维配置缓存不完整");
                }
                File output = safeChild(staging, decodedRelativePath);
                File parent = output.getParentFile();
                if (parent == null) {
                    throw new IOException("三维配置资源解压路径无效");
                }
                ensureDirectory(parent);

                AssetDecoder.Result decoded = AssetDecoder.decode(
                    source,
                    output,
                    metadata.contentEncoding,
                    MAX_DECODED_FILE_BYTES,
                    MIN_FREE_AFTER_DECODE_BYTES,
                    this::isClosed
                );
                String expectedSourceSha = entry.getString("sha256").toLowerCase(Locale.ROOT);
                if (decoded.sourceSize != entry.getLong("size")
                    || !decoded.sourceSha256.equals(expectedSourceSha)) {
                    throw new IOException("三维配置资源SHA-256校验失败");
                }
                if (Long.MAX_VALUE - decodedTotal < decoded.decodedSize) {
                    throw new IOException("三维配置资源解压后总大小超限");
                }
                decodedTotal += decoded.decodedSize;
                if (decodedTotal > MAX_DECODED_TOTAL_BYTES) {
                    throw new IOException("三维配置资源解压后总大小超限");
                }

                JSONObject identityEntry = new JSONObject();
                identityEntry.put("path", relativePath);
                identityEntry.put("decoded_path", decodedRelativePath);
                identityEntry.put("encoding", metadata.contentEncoding);
                identityEntry.put("source_size", decoded.sourceSize);
                identityEntry.put("source_sha256", decoded.sourceSha256);
                identityEntry.put("decoded_size", decoded.decodedSize);
                identityEntry.put("decoded_sha256", decoded.decodedSha256);
                identityEntry.put("decoded_last_modified", output.lastModified());
                identityFiles.put(identityEntry);
            }

            identityManifest.put("schema", IDENTITY_SCHEMA_VERSION);
            identityManifest.put("source_version", manifest.getString("version"));
            identityManifest.put("decoded_total_bytes", decodedTotal);
            identityManifest.put("files", identityFiles);
            writeJson(new File(staging, IDENTITY_MANIFEST), identityManifest);
            validateIdentityDirectory(staging, manifest, false);
            moveDirectoryAtomically(staging, identityRoot);
            publishPreparationProgress(1.0, "");
        } catch (IOException | JSONException | RuntimeException error) {
            deleteRecursively(staging);
            throw error;
        }
    }

    private void validateIdentityPackage(
        File packageRoot,
        JSONObject manifest,
        boolean verifyChecksum
    ) throws IOException, JSONException {
        if (compressedFileCount(manifest) == 0) {
            return;
        }
        validateIdentityDirectory(
            new File(packageRoot, IDENTITY_DIRECTORY),
            manifest,
            verifyChecksum
        );
    }

    private void validateIdentityDirectory(
        File identityRoot,
        JSONObject sourceManifest,
        boolean verifyChecksum
    ) throws IOException, JSONException {
        if (!identityRoot.isDirectory()) {
            throw new IOException("三维配置解压缓存不存在");
        }
        File manifestFile = new File(identityRoot, IDENTITY_MANIFEST);
        if (!manifestFile.isFile()) {
            throw new IOException("三维配置解压缓存清单不存在");
        }
        JSONObject identityManifest;
        try (InputStream stream = new FileInputStream(manifestFile)) {
            identityManifest = new JSONObject(
                new String(readLimited(stream, MAX_MANIFEST_BYTES), StandardCharsets.UTF_8)
            );
        }
        if (identityManifest.getInt("schema") != IDENTITY_SCHEMA_VERSION
            || !sourceManifest.getString("version").equals(
                identityManifest.getString("source_version")
            )) {
            throw new IOException("三维配置解压缓存版本不一致");
        }

        Map<String, JSONObject> identityEntries = new HashMap<>();
        JSONArray storedFiles = identityManifest.getJSONArray("files");
        for (int index = 0; index < storedFiles.length(); index += 1) {
            JSONObject entry = storedFiles.getJSONObject(index);
            String path = entry.getString("path");
            validateManifestAssetPath(path);
            if (identityEntries.put(path, entry) != null) {
                throw new IOException("三维配置解压缓存包含重复路径");
            }
        }

        long decodedTotal = 0L;
        int expectedCount = 0;
        JSONArray sourceFiles = sourceManifest.getJSONArray("files");
        for (int index = 0; index < sourceFiles.length(); index += 1) {
            JSONObject sourceEntry = sourceFiles.getJSONObject(index);
            String path = sourceEntry.getString("path");
            ContentMetadata metadata = contentMetadata(path);
            if (metadata.contentEncoding == null) {
                continue;
            }
            expectedCount += 1;
            JSONObject identityEntry = identityEntries.get(path);
            if (identityEntry == null) {
                throw new IOException("三维配置解压缓存不完整");
            }
            String expectedDecodedPath = decodedRelativePath(path, metadata.contentEncoding);
            String decodedPath = identityEntry.getString("decoded_path");
            validateRelativePath(decodedPath);
            if (!expectedDecodedPath.equals(decodedPath)
                || !metadata.contentEncoding.equals(identityEntry.getString("encoding"))
                || identityEntry.getLong("source_size") != sourceEntry.getLong("size")
                || !sourceEntry.getString("sha256").equalsIgnoreCase(
                    identityEntry.getString("source_sha256")
                )) {
                throw new IOException("三维配置解压缓存来源不一致");
            }
            long decodedSize = identityEntry.getLong("decoded_size");
            long decodedLastModified = identityEntry.getLong("decoded_last_modified");
            String decodedSha = identityEntry.getString("decoded_sha256").toLowerCase(Locale.ROOT);
            if (decodedSize < 0L || decodedSize > MAX_DECODED_FILE_BYTES
                || decodedLastModified <= 0L
                || !decodedSha.matches("[0-9a-f]{64}")) {
                throw new IOException("三维配置解压缓存元数据无效");
            }
            if (Long.MAX_VALUE - decodedTotal < decodedSize) {
                throw new IOException("三维配置解压缓存总大小超限");
            }
            decodedTotal += decodedSize;
            if (decodedTotal > MAX_DECODED_TOTAL_BYTES) {
                throw new IOException("三维配置解压缓存总大小超限");
            }
            File decodedFile = safeChild(identityRoot, decodedPath);
            if (!decodedFile.isFile()
                || decodedFile.length() != decodedSize
                || decodedFile.lastModified() != decodedLastModified) {
                throw new IOException("三维配置解压缓存不完整");
            }
            if (verifyChecksum && !sha256(decodedFile).equals(decodedSha)) {
                throw new IOException("三维配置解压缓存校验失败");
            }
        }
        if (identityEntries.size() != expectedCount
            || identityManifest.getLong("decoded_total_bytes") != decodedTotal) {
            throw new IOException("三维配置解压缓存清单不一致");
        }
    }

    private static int compressedFileCount(JSONObject manifest) throws JSONException {
        int count = 0;
        JSONArray files = manifest.getJSONArray("files");
        for (int index = 0; index < files.length(); index += 1) {
            if (contentMetadata(files.getJSONObject(index).getString("path")).contentEncoding != null) {
                count += 1;
            }
        }
        return count;
    }

    private static String decodedRelativePath(String path, String encoding) throws IOException {
        String suffix = "br".equals(encoding) ? ".br" : "gzip".equals(encoding) ? ".gz" : "";
        if (suffix.isEmpty() || !path.endsWith(suffix) || path.length() <= suffix.length()) {
            throw new IOException("三维配置资源压缩扩展名无效");
        }
        return path.substring(0, path.length() - suffix.length());
    }

    private WebResourceResponse bundledResponse(String relativePath) throws IOException {
        if (relativePath.isEmpty()) {
            relativePath = "index.html";
        }
        validateRelativePath(relativePath);
        AssetManager assets = context.getAssets();
        InputStream stream = assets.open("web/" + relativePath, AssetManager.ACCESS_STREAMING);
        boolean immutable = relativePath.startsWith("assets/") || relativePath.startsWith("icons/");
        return successResponse(relativePath, stream, immutable, false);
    }

    private WebResourceResponse cachedResponse(String requestPath) throws IOException {
        int separator = requestPath.indexOf('/');
        if (separator <= 0 || separator == requestPath.length() - 1) {
            throw new IOException("三维配置资源地址无效");
        }
        String version = requestPath.substring(0, separator);
        String relativePath = requestPath.substring(separator + 1);
        validateVersion(version);
        validateRelativePath(relativePath);
        File packageRoot = versionDirectory(version);
        File file = safeChild(packageRoot, relativePath);
        if (!file.isFile()) {
            throw new IOException("三维配置资源不存在");
        }
        ContentMetadata metadata = contentMetadata(relativePath);
        boolean decoded = metadata.contentEncoding != null;
        if (decoded) {
            file = safeChild(
                new File(packageRoot, IDENTITY_DIRECTORY),
                decodedRelativePath(relativePath, metadata.contentEncoding)
            );
            if (!file.isFile()) {
                throw new IOException("三维配置解压缓存尚未就绪");
            }
        }
        return successResponse(
            relativePath,
            new FileInputStream(file),
            true,
            decoded
        );
    }

    private WebResourceResponse successResponse(
        String path,
        InputStream stream,
        boolean immutable,
        boolean contentDecoded
    ) {
        ContentMetadata metadata = contentMetadata(path);
        Map<String, String> headers = new HashMap<>();
        headers.put("Access-Control-Allow-Origin", "*");
        headers.put("Cross-Origin-Resource-Policy", "cross-origin");
        headers.put("X-Content-Type-Options", "nosniff");
        headers.put(
            "Cache-Control",
            immutable ? "public, max-age=31536000, immutable" : "no-cache"
        );
        if (metadata.contentEncoding != null && !contentDecoded) {
            headers.put("Content-Encoding", metadata.contentEncoding);
        }
        // Chromium derives one Content-Length from FileInputStream.available().
        return new WebResourceResponse(
            metadata.mimeType,
            metadata.characterEncoding,
            200,
            "OK",
            headers,
            stream
        );
    }

    private static WebResourceResponse errorResponse(int statusCode, String reason) {
        Map<String, String> headers = new HashMap<>();
        headers.put("Cache-Control", "no-store");
        return new WebResourceResponse(
            "text/plain",
            "UTF-8",
            statusCode,
            reason,
            headers,
            new ByteArrayInputStream(reason.getBytes(StandardCharsets.UTF_8))
        );
    }

    private JSONObject readyState(JSONObject manifest, String version, String message) {
        JSONObject state = new JSONObject();
        try {
            state.put("status", "ready");
            state.put("progress", 1.0);
            state.put("source", "android_cache");
            state.put("asset_base", APP_ASSET_ORIGIN + VEHICLE_PREFIX + Uri.encode(version) + "/");
            state.put("manifest", new JSONObject(manifest.toString()));
            if (message != null) {
                state.put("message", message);
            }
            return state;
        } catch (JSONException error) {
            throw new IllegalStateException(error);
        }
    }

    private static JSONObject status(String status, double progress, String message) {
        JSONObject state = new JSONObject();
        try {
            state.put("status", status);
            state.put("progress", progress);
            state.put("message", message);
            return state;
        } catch (JSONException error) {
            throw new IllegalStateException(error);
        }
    }

    private void publishProgress(double progress, String relativePath) {
        synchronized (lock) {
            if (closed) {
                return;
            }
            setStateLocked(status(
                "downloading",
                Math.max(0.0, Math.min(1.0, progress)),
                "正在下载 " + relativePath
            ));
        }
        notifyState();
    }

    private void publishPreparationProgress(double progress, String relativePath) {
        synchronized (lock) {
            if (closed) {
                return;
            }
            String message = relativePath.isEmpty()
                ? "三维配置资源准备完成"
                : "正在解压 " + relativePath;
            setStateLocked(status(
                "preparing",
                Math.max(0.0, Math.min(1.0, progress)),
                message
            ));
        }
        notifyState();
    }

    private void setStateLocked(JSONObject state) {
        stateJson = state.toString();
    }

    private void notifyState() {
        String snapshot;
        synchronized (lock) {
            if (closed) {
                return;
            }
            snapshot = stateJson;
        }
        if (listener != null) {
            listener.onStateChanged(snapshot);
        }
    }

    private boolean isClosed() {
        synchronized (lock) {
            return closed;
        }
    }

    private HttpURLConnection openConnection(URL url) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setConnectTimeout(CONNECT_TIMEOUT_MS);
        connection.setReadTimeout(READ_TIMEOUT_MS);
        connection.setUseCaches(false);
        connection.setInstanceFollowRedirects(false);
        connection.setRequestMethod("GET");
        connection.setRequestProperty("Accept-Encoding", "identity");
        return connection;
    }

    private File versionDirectory(String version) throws IOException {
        validateVersion(version);
        return safeChild(versionsRoot, version);
    }

    private static File safeChild(File root, String relativePath) throws IOException {
        validateRelativePath(relativePath);
        File candidate = new File(root, relativePath);
        String rootPath = root.getCanonicalPath();
        String candidatePath = candidate.getCanonicalPath();
        if (!candidatePath.startsWith(rootPath + File.separator)) {
            throw new IOException("三维配置资源路径越界");
        }
        return candidate;
    }

    private static void validateVersion(String version) throws IOException {
        if (version == null || !version.matches("[A-Za-z0-9._-]{1,128}")) {
            throw new IOException("三维配置资源版本无效");
        }
    }

    private static void validateRelativePath(String path) throws IOException {
        if (path == null || path.isEmpty() || path.startsWith("/") || path.contains("\\")) {
            throw new IOException("三维配置资源路径无效");
        }
        for (int index = 0; index < path.length(); index += 1) {
            if (path.charAt(index) < 32) {
                throw new IOException("三维配置资源路径无效");
            }
        }
        for (String part : path.split("/", -1)) {
            if (part.isEmpty() || part.startsWith(".")) {
                throw new IOException("三维配置资源路径无效");
            }
        }
    }

    private static void validateManifestAssetPath(String path) throws IOException {
        validateRelativePath(path);
        String firstPart = path.split("/", 2)[0];
        if (IDENTITY_DIRECTORY.equals(firstPart)
            || firstPart.startsWith(IDENTITY_STAGING_PREFIX)) {
            throw new IOException("三维配置资源路径占用App保留目录");
        }
    }

    private static void ensureDirectory(File directory) throws IOException {
        if (!directory.isDirectory() && !directory.mkdirs()) {
            throw new IOException("无法创建三维配置资源目录");
        }
    }

    private static String trimTrailingSlash(String value) {
        String result = value;
        while (result.endsWith("/")) {
            result = result.substring(0, result.length() - 1);
        }
        return result;
    }

    private static String encodePath(String path) {
        StringBuilder encoded = new StringBuilder();
        String[] parts = path.split("/");
        for (int index = 0; index < parts.length; index += 1) {
            if (index > 0) {
                encoded.append('/');
            }
            encoded.append(Uri.encode(parts[index]));
        }
        return encoded.toString();
    }

    private static byte[] readLimited(InputStream stream, int maximumBytes) throws IOException {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        byte[] buffer = new byte[16 * 1024];
        int count;
        int total = 0;
        while ((count = stream.read(buffer)) != -1) {
            total += count;
            if (total > maximumBytes) {
                throw new IOException("三维配置资源清单过大");
            }
            output.write(buffer, 0, count);
        }
        return output.toByteArray();
    }

    private static void writeManifest(File root, JSONObject manifest) throws IOException {
        writeJson(new File(root, STORED_MANIFEST), manifest);
    }

    private static void writeJson(File file, JSONObject value) throws IOException {
        try (FileOutputStream stream = new FileOutputStream(file)) {
            stream.write(value.toString().getBytes(StandardCharsets.UTF_8));
            stream.getFD().sync();
        }
    }

    private static void moveDirectoryAtomically(File source, File destination) throws IOException {
        try {
            Files.move(
                source.toPath(),
                destination.toPath(),
                StandardCopyOption.ATOMIC_MOVE
            );
        } catch (AtomicMoveNotSupportedException error) {
            if (!source.renameTo(destination)) {
                throw new IOException("无法原子切换三维配置解压缓存", error);
            }
        }
    }

    private static String sha256(File file) throws IOException {
        MessageDigest digest = sha256Digest();
        try (InputStream stream = new FileInputStream(file)) {
            byte[] buffer = new byte[BUFFER_SIZE];
            int count;
            while ((count = stream.read(buffer)) != -1) {
                digest.update(buffer, 0, count);
            }
        }
        return hex(digest.digest());
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

    private static ContentMetadata contentMetadata(String path) {
        String sourceName = path;
        String contentEncoding = null;
        if (sourceName.endsWith(".br")) {
            sourceName = sourceName.substring(0, sourceName.length() - 3);
            contentEncoding = "br";
        } else if (sourceName.endsWith(".gz")) {
            sourceName = sourceName.substring(0, sourceName.length() - 3);
            contentEncoding = "gzip";
        }

        String lowerName = sourceName.toLowerCase(Locale.ROOT);
        String mimeType;
        if (lowerName.endsWith(".wasm")) {
            mimeType = "application/wasm";
        } else if (lowerName.endsWith(".data")) {
            mimeType = "application/octet-stream";
        } else if (lowerName.endsWith(".js")) {
            mimeType = "application/javascript";
        } else if (lowerName.endsWith(".json")) {
            mimeType = "application/json";
        } else if (lowerName.endsWith(".webmanifest")) {
            mimeType = "application/manifest+json";
        } else if (lowerName.endsWith(".svg")) {
            mimeType = "image/svg+xml";
        } else {
            int dot = sourceName.lastIndexOf('.');
            String extension = dot >= 0 ? sourceName.substring(dot + 1) : "";
            mimeType = MimeTypeMap.getSingleton().getMimeTypeFromExtension(extension);
            if (mimeType == null) {
                mimeType = "application/octet-stream";
            }
        }
        String characterEncoding = mimeType.startsWith("text/")
            || "application/javascript".equals(mimeType)
            || "application/json".equals(mimeType)
            || "application/manifest+json".equals(mimeType)
            || "image/svg+xml".equals(mimeType)
            ? "UTF-8"
            : null;
        return new ContentMetadata(mimeType, characterEncoding, contentEncoding);
    }

    private void cleanupOldVersions(String active, String previous, File activeDirectory) {
        File[] versions = versionsRoot.listFiles();
        if (versions != null) {
            for (File version : versions) {
                if (version.equals(activeDirectory)
                    || version.getName().equals(active)
                    || (previous != null && version.getName().equals(previous))) {
                    continue;
                }
                deleteRecursively(version);
            }
        }
        cleanupAbandonedStaging();
    }

    private void cleanupAbandonedStaging() {
        File[] stagingDirectories = stagingRoot.listFiles();
        if (stagingDirectories != null) {
            for (File staging : stagingDirectories) {
                deleteRecursively(staging);
            }
        }
    }

    private static void cleanupIdentityStaging(File packageRoot) {
        File[] children = packageRoot.listFiles();
        if (children == null) {
            return;
        }
        for (File child : children) {
            if (child.getName().startsWith(IDENTITY_STAGING_PREFIX)) {
                deleteRecursively(child);
            }
        }
    }

    private static void deleteRecursively(File file) {
        File[] children = file.listFiles();
        if (children != null) {
            for (File child : children) {
                deleteRecursively(child);
            }
        }
        // Best-effort cleanup only; an inactive package is never served by its descriptor.
        file.delete();
    }

    private final class Progress {
        private final long total;
        private long downloaded;
        private long lastNotification;

        Progress(long total) {
            this.total = Math.max(1L, total);
        }

        void add(int count, String relativePath) {
            downloaded += count;
            long now = android.os.SystemClock.elapsedRealtime();
            if (downloaded == total || now - lastNotification >= PROGRESS_INTERVAL_MS) {
                lastNotification = now;
                publishProgress((double) downloaded / (double) total, relativePath);
            }
        }
    }

    private static final class ContentMetadata {
        final String mimeType;
        final String characterEncoding;
        final String contentEncoding;

        ContentMetadata(String mimeType, String characterEncoding, String contentEncoding) {
            this.mimeType = mimeType;
            this.characterEncoding = characterEncoding;
            this.contentEncoding = contentEncoding;
        }
    }
}
