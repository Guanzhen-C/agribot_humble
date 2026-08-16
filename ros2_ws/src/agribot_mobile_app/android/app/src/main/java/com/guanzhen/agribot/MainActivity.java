package com.guanzhen.agribot;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.graphics.Bitmap;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.webkit.CookieManager;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class MainActivity extends Activity {
    private static final String PREFERENCES = "agribot_mobile";
    private static final String GATEWAY_URL_KEY = "gateway_url";
    private static final int SETTINGS_MENU_ID = 1001;
    private static final long RETRY_DELAY_MS = 5000L;
    private static final int CONNECT_TIMEOUT_MS = 2500;
    private static final String BUNDLED_UI_URL = "file:///android_asset/web/index.html";

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final ExecutorService gatewayProbe = Executors.newSingleThreadExecutor();
    private WebView webView;
    private View offlinePanel;
    private ProgressBar progressBar;
    private TextView offlineMessage;
    private String gatewayUrl;
    private boolean showingBundledUi;
    private boolean probeInFlight;
    private boolean destroyed;
    private final Runnable retryConnection = this::checkGateway;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        gatewayUrl = getSharedPreferences(PREFERENCES, MODE_PRIVATE).getString(
            GATEWAY_URL_KEY,
            BuildConfig.DEFAULT_GATEWAY_URL
        );
        bindViews();
        configureWebView();
        updateActionBar();
        showBundledUi();
        checkGateway();
    }

    private void bindViews() {
        webView = findViewById(R.id.gateway_webview);
        offlinePanel = findViewById(R.id.offline_panel);
        progressBar = findViewById(R.id.loading_progress);
        offlineMessage = findViewById(R.id.offline_message);

        Button retryButton = findViewById(R.id.retry_button);
        Button configureButton = findViewById(R.id.configure_button);
        retryButton.setOnClickListener(view -> checkGateway());
        configureButton.setOnClickListener(view -> showGatewayDialog());
    }

    @SuppressLint("SetJavaScriptEnabled")
    private void configureWebView() {
        WebSettings settings = webView.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setDatabaseEnabled(true);
        settings.setAllowFileAccess(true);
        settings.setAllowFileAccessFromFileURLs(true);
        settings.setAllowUniversalAccessFromFileURLs(false);
        settings.setAllowContentAccess(false);
        settings.setMixedContentMode(WebSettings.MIXED_CONTENT_COMPATIBILITY_MODE);
        settings.setCacheMode(WebSettings.LOAD_DEFAULT);
        settings.setMediaPlaybackRequiresUserGesture(true);

        CookieManager.getInstance().setAcceptCookie(true);
        webView.setWebChromeClient(new WebChromeClient());
        webView.setWebViewClient(new GatewayWebViewClient());
        WebView.setWebContentsDebuggingEnabled(BuildConfig.DEBUG);
    }

    private void loadGateway() {
        handler.removeCallbacks(retryConnection);
        showingBundledUi = false;
        progressBar.setVisibility(View.VISIBLE);
        offlinePanel.setVisibility(View.GONE);
        webView.loadUrl(gatewayUrl);
    }

    private void showBundledUi() {
        progressBar.setVisibility(View.GONE);
        offlinePanel.setVisibility(View.GONE);
        if (!showingBundledUi) {
            showingBundledUi = true;
            webView.loadUrl(BUNDLED_UI_URL);
        }
        scheduleRetry();
    }

    private void showNativeOffline(String detail) {
        progressBar.setVisibility(View.GONE);
        offlinePanel.setVisibility(View.VISIBLE);
        offlineMessage.setText(getString(R.string.connection_failed, gatewayUrl, detail));
        scheduleRetry();
    }

    private void scheduleRetry() {
        handler.removeCallbacks(retryConnection);
        handler.postDelayed(retryConnection, RETRY_DELAY_MS);
    }

    private void checkGateway() {
        handler.removeCallbacks(retryConnection);
        if (destroyed || probeInFlight) {
            return;
        }
        probeInFlight = true;
        final String checkedUrl = gatewayUrl;
        gatewayProbe.execute(() -> {
            boolean reachable = gatewayIsReachable(checkedUrl);
            handler.post(() -> {
                probeInFlight = false;
                if (destroyed || !checkedUrl.equals(gatewayUrl)) {
                    return;
                }
                if (reachable) {
                    loadGateway();
                } else {
                    showBundledUi();
                }
            });
        });
    }

    static boolean gatewayIsReachable(String baseUrl) {
        HttpURLConnection connection = null;
        try {
            connection = (HttpURLConnection) new URL(baseUrl + "/api/v1/state").openConnection();
            connection.setConnectTimeout(CONNECT_TIMEOUT_MS);
            connection.setReadTimeout(CONNECT_TIMEOUT_MS);
            connection.setUseCaches(false);
            connection.setRequestMethod("GET");
            return connection.getResponseCode() == HttpURLConnection.HTTP_OK;
        } catch (IOException error) {
            return false;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    private void showGatewayDialog() {
        EditText input = new EditText(this);
        input.setSingleLine(true);
        input.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        input.setText(gatewayUrl);
        input.setSelectAllOnFocus(true);
        int padding = Math.round(20 * getResources().getDisplayMetrics().density);
        FrameLayout inputContainer = new FrameLayout(this);
        inputContainer.setPadding(padding, 0, padding, 0);
        inputContainer.addView(
            input,
            new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT
            )
        );

        AlertDialog dialog = new AlertDialog.Builder(this)
            .setTitle(R.string.gateway_dialog_title)
            .setMessage(R.string.gateway_dialog_message)
            .setView(inputContainer)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton(R.string.connect, null)
            .create();

        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
            .setOnClickListener(view -> {
                try {
                    gatewayUrl = normalizeGatewayUrl(input.getText().toString());
                    getSharedPreferences(PREFERENCES, MODE_PRIVATE)
                        .edit()
                        .putString(GATEWAY_URL_KEY, gatewayUrl)
                        .apply();
                    updateActionBar();
                    dialog.dismiss();
                    showBundledUi();
                    checkGateway();
                } catch (IllegalArgumentException exception) {
                    input.setError(exception.getMessage());
                }
            }));
        dialog.show();
    }

    static String normalizeGatewayUrl(String value) {
        String candidate = value.trim();
        if (!candidate.contains("://")) {
            candidate = "http://" + candidate;
        }

        try {
            URI uri = new URI(candidate);
            String scheme = uri.getScheme() == null
                ? "http"
                : uri.getScheme().toLowerCase(Locale.ROOT);
            if (!("http".equals(scheme) || "https".equals(scheme))) {
                throw new IllegalArgumentException("仅支持HTTP或HTTPS地址");
            }
            if (uri.getHost() == null || uri.getHost().isEmpty()) {
                throw new IllegalArgumentException("请输入有效的RDK地址");
            }
            int port = uri.getPort();
            if (port < 0 && "http".equals(scheme)) {
                port = 8088;
            }
            return new URI(scheme, null, uri.getHost(), port, null, null, null).toString();
        } catch (URISyntaxException exception) {
            throw new IllegalArgumentException("请输入有效的RDK地址");
        }
    }

    private void updateActionBar() {
        if (getActionBar() != null) {
            getActionBar().setTitle(R.string.app_name);
            getActionBar().setSubtitle(Uri.parse(gatewayUrl).getAuthority());
        }
    }

    private boolean belongsToGateway(Uri uri) {
        Uri gateway = Uri.parse(gatewayUrl);
        return gateway.getScheme() != null
            && gateway.getScheme().equalsIgnoreCase(uri.getScheme())
            && gateway.getHost() != null
            && gateway.getHost().equalsIgnoreCase(uri.getHost())
            && effectivePort(gateway) == effectivePort(uri);
    }

    private static boolean isBundledUrl(String url) {
        return url != null && url.startsWith("file:///android_asset/web/");
    }

    private static int effectivePort(Uri uri) {
        if (uri.getPort() >= 0) {
            return uri.getPort();
        }
        return "https".equalsIgnoreCase(uri.getScheme()) ? 443 : 80;
    }

    private void openExternal(Uri uri) {
        try {
            startActivity(new Intent(Intent.ACTION_VIEW, uri));
        } catch (ActivityNotFoundException exception) {
            Toast.makeText(this, R.string.no_browser, Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuItem settings = menu.add(Menu.NONE, SETTINGS_MENU_ID, Menu.NONE, R.string.gateway_settings);
        settings.setIcon(android.R.drawable.ic_menu_preferences);
        settings.setShowAsAction(MenuItem.SHOW_AS_ACTION_IF_ROOM);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == SETTINGS_MENU_ID) {
            showGatewayDialog();
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onBackPressed() {
        if (webView.canGoBack()) {
            webView.goBack();
        } else {
            super.onBackPressed();
        }
    }

    @Override
    protected void onDestroy() {
        destroyed = true;
        handler.removeCallbacksAndMessages(null);
        gatewayProbe.shutdownNow();
        webView.stopLoading();
        webView.destroy();
        super.onDestroy();
    }

    private final class GatewayWebViewClient extends WebViewClient {
        @Override
        public void onPageStarted(WebView view, String url, Bitmap favicon) {
            if (!isBundledUrl(url)) {
                progressBar.setVisibility(View.VISIBLE);
            }
        }

        @Override
        public void onPageFinished(WebView view, String url) {
            if (!url.equals(view.getUrl())) {
                return;
            }
            progressBar.setVisibility(View.GONE);
            offlinePanel.setVisibility(View.GONE);
            if (isBundledUrl(url)) {
                showingBundledUi = true;
                scheduleRetry();
            } else if (belongsToGateway(Uri.parse(url))) {
                showingBundledUi = false;
                handler.removeCallbacks(retryConnection);
            }
        }

        @Override
        public void onReceivedError(
            WebView view,
            WebResourceRequest request,
            WebResourceError error
        ) {
            if (request.isForMainFrame()) {
                if (isBundledUrl(request.getUrl().toString())) {
                    showNativeOffline(error.getDescription().toString());
                } else {
                    showBundledUi();
                }
            }
        }

        @Override
        public void onReceivedHttpError(
            WebView view,
            WebResourceRequest request,
            WebResourceResponse errorResponse
        ) {
            if (request.isForMainFrame()) {
                showBundledUi();
            }
        }

        @Override
        public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
            if (isBundledUrl(request.getUrl().toString()) || belongsToGateway(request.getUrl())) {
                return false;
            }
            openExternal(request.getUrl());
            return true;
        }

    }
}
