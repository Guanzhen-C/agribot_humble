package com.guanzhen.agribot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public final class MainActivityTest {
    @Test
    public void addsTheDefaultGatewayPort() {
        assertEquals(
            "http://192.168.100.125:8088",
            MainActivity.normalizeGatewayUrl("192.168.100.125")
        );
    }

    @Test
    public void preservesAnExplicitPort() {
        assertEquals(
            "http://10.0.0.8:9000",
            MainActivity.normalizeGatewayUrl("http://10.0.0.8:9000/path")
        );
    }

    @Test
    public void preservesHttpsWithoutAddingTheHttpPort() {
        assertEquals(
            "https://robot.local",
            MainActivity.normalizeGatewayUrl("https://robot.local")
        );
    }
}
