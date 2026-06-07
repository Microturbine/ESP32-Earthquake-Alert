#ifndef WEBUI_HTML_H
#define WEBUI_HTML_H

#include <pgmspace.h>

const char WEBUI_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>災害警報レシーバー ダッシュボード</title>
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@300;400;500;600;700&family=Noto+Sans+JP:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-primary: #0a0712;
            --bg-secondary: #130f22;
            --card-bg: rgba(25, 20, 42, 0.55);
            --card-border: rgba(255, 255, 255, 0.08);
            --card-border-hover: rgba(255, 255, 255, 0.16);
            --text-primary: #f4f3f6;
            --text-secondary: #9c97aa;
            --accent: #8b5cf6;
            --accent-hover: #a78bfa;
            --accent-glow: rgba(139, 92, 246, 0.3);
            --danger: #ef4444;
            --danger-glow: rgba(239, 68, 68, 0.4);
            --warning: #ec4899;
            --warning-glow: rgba(236, 72, 153, 0.4);
            --success: #10b981;
            --success-glow: rgba(16, 185, 129, 0.2);
            --info: #06b6d4;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: 'Plus Jakarta Sans', 'Noto Sans JP', sans-serif;
            background-color: var(--bg-primary);
            background-image: 
                radial-gradient(at 10% 20%, rgba(139, 92, 246, 0.15) 0px, transparent 50%),
                radial-gradient(at 90% 80%, rgba(236, 72, 153, 0.1) 0px, transparent 50%);
            background-attachment: fixed;
            color: var(--text-primary);
            min-height: 100vh;
            padding: 1.5rem;
            line-height: 1.5;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
        }

        /* ヘッダー */
        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 1.5rem;
            border-bottom: 1px solid rgba(255, 255, 255, 0.05);
            margin-bottom: 2rem;
        }

        .brand h1 {
            font-size: 1.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #fff 0%, var(--text-secondary) 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            letter-spacing: -0.5px;
        }

        .brand p {
            font-size: 0.8rem;
            color: var(--text-secondary);
            margin-top: 0.2rem;
        }

        .status-badge {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            background: rgba(255, 255, 255, 0.05);
            padding: 0.5rem 1rem;
            border-radius: 9999px;
            font-size: 0.8rem;
            font-weight: 500;
            border: 1px solid var(--card-border);
            backdrop-filter: blur(10px);
        }

        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background-color: var(--danger);
            box-shadow: 0 0 8px var(--danger);
        }

        .status-dot.online {
            background-color: var(--success);
            box-shadow: 0 0 8px var(--success);
            animation: pulse 2s infinite;
        }

        /* メイングリッド */
        .dashboard-grid {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 1.5rem;
        }

        @media (max-width: 900px) {
            .dashboard-grid {
                grid-template-columns: 1fr;
            }
        }

        /* カードの共通スタイル */
        .card {
            background: var(--card-bg);
            border: 1px solid var(--card-border);
            border-radius: 16px;
            padding: 1.5rem;
            backdrop-filter: blur(16px);
            -webkit-backdrop-filter: blur(16px);
            transition: border-color 0.3s ease, box-shadow 0.3s ease;
        }

        .card:hover {
            border-color: var(--card-border-hover);
        }

        .card-title {
            font-size: 1rem;
            font-weight: 600;
            margin-bottom: 1.2rem;
            display: flex;
            align-items: center;
            gap: 0.5rem;
            color: var(--text-primary);
        }

        .card-title svg {
            width: 18px;
            height: 18px;
            stroke: var(--accent);
        }

        /* 警報カードエリア */
        .alerts-section {
            display: flex;
            flex-direction: column;
            gap: 1rem;
        }

        .alert-card {
            border: 1px solid var(--danger);
            background: rgba(239, 68, 68, 0.06);
            border-radius: 12px;
            padding: 1.2rem;
            position: relative;
            overflow: hidden;
            animation: slideIn 0.3s ease-out;
            box-shadow: 0 4px 20px var(--danger-glow);
        }

        .alert-card.test-alert {
            border-color: var(--warning);
            background: rgba(236, 72, 153, 0.06);
            box-shadow: 0 4px 20px var(--warning-glow);
        }

        .alert-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 4px;
            height: 100%;
            background: var(--danger);
        }

        .alert-card.test-alert::before {
            background: var(--warning);
        }

        .alert-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 0.5rem;
        }

        .alert-type-badge {
            font-size: 0.75rem;
            font-weight: 700;
            text-transform: uppercase;
            padding: 0.25rem 0.6rem;
            border-radius: 4px;
            color: #fff;
            background: var(--danger);
            letter-spacing: 0.5px;
        }

        .alert-card.test-alert .alert-type-badge {
            background: var(--warning);
        }

        .alert-expiry {
            font-size: 0.75rem;
            color: var(--text-secondary);
        }

        .alert-body {
            font-size: 1.15rem;
            font-weight: 600;
            color: #fff;
            margin-top: 0.5rem;
            letter-spacing: 0.2px;
            line-height: 1.4;
        }

        /* 待機状態 (警報なし) */
        .standby-state {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            padding: 4rem 2rem;
            text-align: center;
            background: rgba(255, 255, 255, 0.01);
            border: 1px dashed rgba(255, 255, 255, 0.1);
            border-radius: 16px;
        }

        .radar-wave {
            position: relative;
            width: 80px;
            height: 80px;
            border-radius: 50%;
            background: rgba(16, 185, 129, 0.05);
            display: flex;
            align-items: center;
            justify-content: center;
            margin-bottom: 1.5rem;
        }

        .radar-wave::before, .radar-wave::after {
            content: '';
            position: absolute;
            width: 100%;
            height: 100%;
            border-radius: 50%;
            border: 1.5px solid var(--success);
            opacity: 0;
            animation: radarPulse 2.5s cubic-bezier(0.215, 0.610, 0.355, 1) infinite;
        }

        .radar-wave::after {
            animation-delay: 1.25s;
        }

        .radar-icon {
            width: 24px;
            height: 24px;
            fill: var(--success);
            filter: drop-shadow(0 0 8px var(--success));
        }

        .standby-state h3 {
            font-size: 1.1rem;
            font-weight: 600;
            color: var(--text-primary);
            margin-bottom: 0.4rem;
        }

        .standby-state p {
            font-size: 0.85rem;
            color: var(--text-secondary);
        }

        /* 受信ステータスグリッド */
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 1rem;
            margin-bottom: 1.5rem;
        }

        .stat-card {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid var(--card-border);
            border-radius: 12px;
            padding: 1.2rem;
            display: flex;
            flex-direction: column;
            gap: 0.4rem;
        }

        .stat-label {
            font-size: 0.75rem;
            font-weight: 500;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .stat-value {
            font-size: 1.5rem;
            font-weight: 700;
            color: #fff;
        }

        .stat-sub {
            font-size: 0.75rem;
            color: var(--text-secondary);
            display: flex;
            align-items: center;
            gap: 0.3rem;
        }

        /* プログレス・メーター */
        .rssi-bar-container {
            width: 100%;
            height: 6px;
            background: rgba(255, 255, 255, 0.08);
            border-radius: 9999px;
            overflow: hidden;
            margin-top: 0.4rem;
        }

        .rssi-bar {
            height: 100%;
            width: 0%;
            background: linear-gradient(90deg, var(--warning), var(--success));
            border-radius: 9999px;
            transition: width 0.5s cubic-bezier(0.4, 0, 0.2, 1);
        }

        /* フォームとコントロール */
        .control-tabs {
            display: flex;
            gap: 0.5rem;
            background: rgba(255, 255, 255, 0.03);
            padding: 0.3rem;
            border-radius: 8px;
            margin-bottom: 1.2rem;
            border: 1px solid var(--card-border);
        }

        .tab-btn {
            flex: 1;
            background: transparent;
            border: none;
            color: var(--text-secondary);
            padding: 0.5rem;
            border-radius: 6px;
            cursor: pointer;
            font-size: 0.85rem;
            font-weight: 500;
            transition: all 0.2s ease;
        }

        .tab-btn.active {
            background: var(--accent);
            color: #fff;
            box-shadow: 0 2px 8px var(--accent-glow);
        }

        .tab-content {
            display: none;
            animation: fadeIn 0.2s ease;
        }

        .tab-content.active {
            display: block;
        }

        .form-group {
            margin-bottom: 1rem;
        }

        .form-group label {
            display: block;
            font-size: 0.8rem;
            font-weight: 500;
            color: var(--text-secondary);
            margin-bottom: 0.4rem;
        }

        .input-row {
            display: flex;
            gap: 0.5rem;
        }

        .form-control {
            flex: 1;
            background: rgba(255, 255, 255, 0.04);
            border: 1px solid var(--card-border);
            border-radius: 8px;
            padding: 0.6rem 0.8rem;
            color: #fff;
            font-family: inherit;
            font-size: 0.9rem;
            transition: border-color 0.2s ease;
        }

        .form-control:focus {
            outline: none;
            border-color: var(--accent);
            background: rgba(255, 255, 255, 0.08);
        }

        /* レンジスライダーのカスタマイズ */
        .slider-container {
            display: flex;
            align-items: center;
            gap: 1rem;
        }

        .slider-container input[type="range"] {
            flex: 1;
            -webkit-appearance: none;
            height: 6px;
            border-radius: 9999px;
            background: rgba(255, 255, 255, 0.08);
            outline: none;
        }

        .slider-container input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: var(--accent);
            cursor: pointer;
            transition: transform 0.1s ease;
            box-shadow: 0 0 8px var(--accent-glow);
        }

        .slider-container input[type="range"]::-webkit-slider-thumb:hover {
            transform: scale(1.2);
        }

        .slider-val {
            font-size: 0.9rem;
            font-weight: 600;
            min-width: 20px;
            text-align: right;
        }

        /* ボタン */
        .btn {
            background: var(--accent);
            color: #fff;
            border: none;
            border-radius: 8px;
            padding: 0.6rem 1.2rem;
            font-size: 0.85rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s ease;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 0.4rem;
        }

        .btn:hover {
            background: var(--accent-hover);
            box-shadow: 0 0 12px var(--accent-glow);
        }

        .btn:active {
            transform: scale(0.97);
        }

        .btn-secondary {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid var(--card-border);
            color: var(--text-primary);
        }

        .btn-secondary:hover {
            background: rgba(255, 255, 255, 0.1);
            box-shadow: none;
        }

        .btn-danger {
            background: var(--danger);
        }

        .btn-danger:hover {
            background: #f87171;
            box-shadow: 0 0 12px var(--danger-glow);
        }

        .btn-full {
            width: 100%;
        }

        .btn-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0.5rem;
            margin-bottom: 0.8rem;
        }

        /* アニメーション */
        @keyframes pulse {
            0% { transform: scale(1); opacity: 1; }
            50% { transform: scale(1.15); opacity: 0.6; }
            100% { transform: scale(1); opacity: 1; }
        }

        @keyframes radarPulse {
            0% { transform: scale(0.6); opacity: 0; }
            50% { opacity: 0.4; }
            100% { transform: scale(1.5); opacity: 0; }
        }

        @keyframes slideIn {
            from { transform: translateY(10px); opacity: 0; }
            to { transform: translateY(0); opacity: 1; }
        }

        @keyframes fadeIn {
            from { opacity: 0; }
            to { opacity: 1; }
        }
    </style>
</head>
<body>
    <div class="container">
        <!-- ヘッダー -->
        <header>
            <div class="brand">
                <h1>Disaster Alert Receiver</h1>
                <p>みちびき L1S (QZSS) &amp; 地上 FM EWS ハイブリッドデコーダ</p>
            </div>
            <div class="status-badge">
                <div id="connection-dot" class="status-dot"></div>
                <span id="connection-text">切断中</span>
            </div>
        </header>

        <!-- メインコンテンツ -->
        <div class="dashboard-grid">
            <!-- 左カラム: アラート情報とデバイスステータス -->
            <div style="display: flex; flex-direction: column; gap: 1.5rem;">
                
                <!-- 受信アラート -->
                <div class="card">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3Z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>
                        アクティブな防災警報
                    </div>
                    <div id="alerts-container" class="alerts-section">
                        <!-- 動的に警報カードが追加されます -->
                        <div class="standby-state">
                            <div class="radar-wave">
                                <svg class="radar-icon" viewBox="0 0 24 24"><path d="M12 22c5.523 0 10-4.477 10-10S17.523 2 12 2 2 6.477 2 12s4.477 10 10 10zm0-2a8 8 0 1 1 0-16 8 8 0 0 1 0 16zm0-12a4 4 0 1 0 0 8 4 4 0 0 0 0-8zm0 2a2 2 0 1 1 0 4 2 2 0 0 1 0-4z"/></svg>
                            </div>
                            <h3>システム監視中 - 待機状態</h3>
                            <p>現在、検知された災害警報はありません。</p>
                        </div>
                    </div>
                </div>

                <!-- ステータスメーター -->
                <div class="card">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"/><line x1="9" y1="17" x2="9" y2="9"/><line x1="15" y1="17" x2="15" y2="13"/></svg>
                        受信機ステータス
                    </div>
                    <div class="stats-grid">
                        <div class="stat-card">
                            <span class="stat-label">FMラジオ周波数</span>
                            <span class="stat-value" id="status-freq">--.- MHz</span>
                            <span class="stat-sub" id="status-rssi-container">
                                RSSI: <span id="status-rssi">0</span>
                                <div class="rssi-bar-container">
                                    <div id="rssi-indicator" class="rssi-bar"></div>
                                </div>
                            </span>
                        </div>
                        <div class="stat-card">
                            <span class="stat-label">GPS時間・接続</span>
                            <span class="stat-value" id="status-time">--:--:--</span>
                            <span class="stat-sub">衛星捕捉数: <span id="status-sats" style="font-weight:600;color:var(--info);">0</span></span>
                        </div>
                        <div class="stat-card">
                            <span class="stat-label">デコーダ状態</span>
                            <span class="stat-value" id="status-ews" style="font-size: 1.1rem; line-height: 2rem; color: var(--text-primary);">待機状態</span>
                            <span class="stat-sub">地域コード: <span id="status-region" style="font-weight:600;">0</span></span>
                        </div>
                    </div>
                </div>

            </div>

            <!-- 右カラム: コントロールパネル -->
            <div style="display: flex; flex-direction: column; gap: 1.5rem;">
                
                <!-- 基本設定 -->
                <div class="card">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/></svg>
                        レシーバー設定
                    </div>
                    
                    <div class="control-tabs">
                        <button class="tab-btn active" onclick="switchTab('tab-radio')">受信設定</button>
                        <button class="tab-btn" onclick="switchTab('tab-wifi')">WiFi設定</button>
                    </div>

                    <!-- 受信設定タブ -->
                    <div id="tab-radio" class="tab-content active">
                        <form id="form-radio" onsubmit="submitRadio(event)">
                            <div class="form-group">
                                <label for="input-freq">FM受信周波数 (76.0 - 108.0 MHz)</label>
                                <div class="input-row">
                                    <input type="number" step="0.1" min="76.0" max="108.0" id="input-freq" class="form-control" required placeholder="例: 80.0">
                                    <button type="submit" class="btn">選局</button>
                                </div>
                            </div>
                            <div class="form-group">
                                <label for="slider-vol">音声出力音量 (0 - 15)</label>
                                <div class="slider-container">
                                    <input type="range" min="0" max="15" id="slider-vol" oninput="updateVolValue(this.value)" onchange="sendVolume(this.value)">
                                    <span class="slider-val" id="vol-value">1</span>
                                </div>
                            </div>
                            <div class="form-group">
                                <label for="input-region">自地域コード (0: 無効, 1-47: 都道府県)</label>
                                <div class="input-row">
                                    <input type="number" min="0" max="99" id="input-region" class="form-control" required placeholder="例: 13 (東京)">
                                    <button type="button" onclick="submitRegion()" class="btn btn-secondary">設定</button>
                                </div>
                                <span style="font-size:0.7rem;color:var(--text-secondary);margin-top:0.3rem;display:block;">
                                    ※みちびき災危通報・EWS地域判定用フィルタコード
                                </span>
                            </div>
                        </form>
                    </div>

                    <!-- WiFi設定タブ -->
                    <div id="tab-wifi" class="tab-content">
                        <form id="form-wifi" onsubmit="submitWiFi(event)">
                            <div class="form-group">
                                <label for="input-ssid">WiFi SSID</label>
                                <input type="text" id="input-ssid" class="form-control" placeholder="接続先SSID">
                            </div>
                            <div class="form-group">
                                <label for="input-pass">パスワード</label>
                                <input type="password" id="input-pass" class="form-control" placeholder="接続パスワード">
                            </div>
                            <button type="submit" class="btn btn-full">WiFi設定を保存して再起動</button>
                            <span style="font-size:0.7rem;color:var(--text-secondary);margin-top:0.5rem;display:block;text-align:center;">
                                ※保存後、デバイスは再起動します
                            </span>
                        </form>
                    </div>
                </div>

                <!-- テスト・デバッグ -->
                <div class="card">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="m8 2 8 8-8 8Z"/><path d="M22 12H2"/><path d="m16 22-8-8 8-8Z"/></svg>
                        テストと制御
                    </div>
                    <div class="btn-grid">
                        <button class="btn btn-secondary btn-danger" onclick="triggerMock('eq')">模擬地震</button>
                        <button class="btn btn-secondary btn-danger" onclick="triggerMock('tsunami')">模擬津波</button>
                    </div>
                    <div class="btn-grid">
                        <button class="btn btn-secondary" style="grid-column: span 2;" onclick="triggerMock('jalert')">模擬 Jアラート (国民保護)</button>
                    </div>
                    
                    <div class="form-group" style="margin-top:0.8rem;">
                        <label for="input-hex">みちびき生パケット(HEX32バイト)</label>
                        <div class="input-row">
                            <input type="text" id="input-hex" class="form-control" style="font-family: monospace; font-size: 0.8rem;" placeholder="533B06... (64文字)">
                            <button type="button" onclick="submitHex()" class="btn btn-secondary">送信</button>
                        </div>
                    </div>

                    <button class="btn btn-danger btn-full" onclick="clearAlerts()" style="margin-top: 0.5rem;">
                        警報の初期化 (Reset)
                    </button>
                </div>

            </div>
        </div>
    </div>

    <script>
        let isAPMode = false;
        let lastIp = "";

        // タブ切り替え
        function switchTab(tabId) {
            document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
            document.querySelectorAll('.tab-content').forEach(content => content.classList.remove('active'));
            
            // ボタンのアクティブ化
            const eventTarget = window.event ? window.event.target : null;
            if (eventTarget) {
                eventTarget.classList.add('active');
            } else {
                // ボタンの自動選択
                document.querySelector(`[onclick="switchTab('${tabId}')"]`).classList.add('active');
            }
            
            document.getElementById(tabId).classList.add('active');
        }

        // 音量表示の更新
        function updateVolValue(val) {
            document.getElementById('vol-value').innerText = val;
        }

        // 状態のポーリング
        async function fetchStatus() {
            try {
                const response = await fetch('/api/status');
                if (!response.ok) throw new Error('API Response Error');
                const data = await response.json();
                
                // コネクション表示
                const dot = document.getElementById('connection-dot');
                const text = document.getElementById('connection-text');
                dot.classList.add('online');
                
                isAPMode = data.wifiMode === "AP";
                lastIp = data.ip || "";
                text.innerText = `${data.wifiMode === "AP" ? "APモード" : "接続中"} (${lastIp})`;

                // FMステータス
                document.getElementById('status-freq').innerText = `${data.freq.toFixed(1)} MHz`;
                document.getElementById('status-rssi').innerText = data.rssi;
                
                const rssiPercent = Math.min(Math.max((data.rssi / 60) * 100, 0), 100);
                document.getElementById('rssi-indicator').style.width = `${rssiPercent}%`;

                // GPSステータス
                document.getElementById('status-time').innerText = data.time;
                document.getElementById('status-sats').innerText = data.svCount;

                // EWS・地域
                let ewsText = "待機状態";
                if (data.ewsState === 1) {
                    ewsText = "FM 警報信号解析中";
                }
                document.getElementById('status-ews').innerText = ewsText;
                document.getElementById('status-region').innerText = data.region;

                // フォームの値を初回のみ設定
                if (!document.activeElement || document.activeElement.tagName !== 'INPUT') {
                    if (document.getElementById('input-freq').value === "") {
                        document.getElementById('input-freq').value = data.freq.toFixed(1);
                    }
                    if (document.getElementById('input-region').value === "") {
                        document.getElementById('input-region').value = data.region;
                    }
                }
                
                // スライダー値同期
                const volSlider = document.getElementById('slider-vol');
                if (document.activeElement !== volSlider) {
                    volSlider.value = data.volume;
                    updateVolValue(data.volume);
                }

                // 警報表示の更新
                const alertsContainer = document.getElementById('alerts-container');
                if (data.alerts && data.alerts.length > 0) {
                    const currentAlertsSignature = data.alerts.map(a => a.text + "|" + a.isTest).join("||");
                    if (alertsContainer.getAttribute('data-signature') !== currentAlertsSignature) {
                        alertsContainer.setAttribute('data-signature', currentAlertsSignature);
                        let html = '';
                        data.alerts.forEach((alert, idx) => {
                            const isTest = alert.isTest;
                            const cardClass = isTest ? 'alert-card test-alert' : 'alert-card';
                            const badgeText = isTest ? '訓練/試験' : '災害警報';
                            
                            html += `
                                <div class="${cardClass}" id="alert-card-${idx}">
                                    <div class="alert-header">
                                        <span class="alert-type-badge">${badgeText}</span>
                                        <span class="alert-expiry" id="alert-expiry-${idx}"></span>
                                    </div>
                                    <div class="alert-body">${alert.text}</div>
                                </div>
                            `;
                        });
                        alertsContainer.innerHTML = html;
                    }
                    
                    // 有効期限の数値のみをインプレースで更新（アニメーションの再発火を防止）
                    data.alerts.forEach((alert, idx) => {
                        const expiryEl = document.getElementById(`alert-expiry-${idx}`);
                        if (expiryEl) {
                            let expiryText = "";
                            if (alert.remaining > 0) {
                                expiryText = `有効期限: 約${alert.remaining}秒`;
                            } else if (alert.remaining === 0) {
                                expiryText = "期限切れ間近";
                            }
                            expiryEl.innerText = expiryText;
                        }
                    });
                } else {
                    if (alertsContainer.getAttribute('data-signature') !== "standby") {
                        alertsContainer.setAttribute('data-signature', "standby");
                        alertsContainer.innerHTML = `
                            <div class="standby-state">
                                <div class="radar-wave">
                                    <svg class="radar-icon" viewBox="0 0 24 24"><path d="M12 22c5.523 0 10-4.477 10-10S17.523 2 12 2 2 6.477 2 12s4.477 10 10 10zm0-2a8 8 0 1 1 0-16 8 8 0 0 1 0 16zm0-12a4 4 0 1 0 0 8 4 4 0 0 0 0-8zm0 2a2 2 0 1 1 0 4 2 2 0 0 1 0-4z"/></svg>
                                </div>
                                <h3>システム監視中 - 待機状態</h3>
                                <p>現在、検知された災害警報はありません。</p>
                            </div>
                        `;
                    }
                }

            } catch (err) {
                console.error(err);
                document.getElementById('connection-dot').classList.remove('online');
                document.getElementById('connection-text').innerText = "切断中 (再接続試行...)";
            }
        }

        // ラジオ設定送信
        async function submitRadio(e) {
            e.preventDefault();
            const freq = parseFloat(document.getElementById('input-freq').value);
            if (freq >= 76.0 && freq <= 108.0) {
                await sendSettings({ freq: freq });
            }
        }

        // 音量送信
        async function sendVolume(vol) {
            await sendSettings({ volume: parseInt(vol) });
        }

        // 地域コード送信
        async function submitRegion() {
            const reg = parseInt(document.getElementById('input-region').value);
            if (reg >= 0 && reg <= 99) {
                await sendSettings({ region: reg });
            }
        }

        // 設定の汎用POST
        async function sendSettings(params) {
            try {
                const formData = new URLSearchParams();
                for (const key in params) {
                    formData.append(key, params[key]);
                }
                const response = await fetch('/api/settings', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: formData
                });
                if (response.ok) {
                    fetchStatus();
                } else {
                    alert('設定の保存に失敗しました');
                }
            } catch (err) {
                alert('通信エラーが発生しました');
            }
        }

        // WiFi設定送信
        async function submitWiFi(e) {
            e.preventDefault();
            const ssid = document.getElementById('input-ssid').value;
            const pass = document.getElementById('input-pass').value;
            
            if (confirm('WiFi設定を保存してデバイスを再起動します。よろしいですか？')) {
                try {
                    const formData = new URLSearchParams();
                    formData.append('ssid', ssid);
                    formData.append('pass', pass);
                    await fetch('/api/settings', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                        body: formData
                    });
                    alert('設定を送信しました。デバイスを再起動します。約10秒後に新しいネットワークでの接続を確認してください。');
                } catch (err) {
                    alert('送信に失敗しました');
                }
            }
        }

        // 模擬アラートトリガー
        async function triggerMock(type) {
            try {
                const formData = new URLSearchParams();
                formData.append('type', type);
                await fetch('/api/test', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: formData
                });
                fetchStatus();
            } catch (err) {
                console.error(err);
            }
        }

        // HEX送信
        async function submitHex() {
            const hex = document.getElementById('input-hex').value.trim();
            if (hex.length !== 64) {
                alert('生パケットデータは64文字の16進数（32バイト）である必要があります。');
                return;
            }
            try {
                const formData = new URLSearchParams();
                formData.append('type', 'hex');
                formData.append('hex', hex);
                await fetch('/api/test', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: formData
                });
                fetchStatus();
            } catch (err) {
                console.error(err);
            }
        }

        // 警報クリア
        async function clearAlerts() {
            try {
                await fetch('/api/clear', { method: 'POST' });
                fetchStatus();
            } catch (err) {
                console.error(err);
            }
        }

        // 起動時および定期更新の開始
        fetchStatus();
        setInterval(fetchStatus, 1500);
    </script>
</body>
</html>
)rawhtml";

#endif // WEBUI_HTML_H
