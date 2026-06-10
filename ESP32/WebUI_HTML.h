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
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="icon" href="/favicon.ico" type="image/x-icon">
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
            --marine: #3b82f6;
            --marine-glow: rgba(59, 130, 246, 0.4);
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

        .alert-card.cancel-alert {
            border-color: var(--success);
            background: rgba(16, 185, 129, 0.06);
            box-shadow: 0 4px 20px var(--success-glow);
        }

        .alert-card.out-of-region {
            border-color: var(--text-secondary);
            background: rgba(156, 151, 170, 0.06);
            box-shadow: none;
        }

        .alert-card.marine-alert {
            border-color: var(--marine);
            background: rgba(59, 130, 246, 0.06);
            box-shadow: 0 4px 20px var(--marine-glow);
        }

        /* 海上警報アコーディオン用スタイル */
        .marine-details {
            width: 100%;
        }
        .marine-summary {
            cursor: pointer;
            outline: none;
            user-select: none;
            display: list-item;
        }
        .marine-summary::-webkit-details-marker {
            color: var(--marine);
        }
        .alert-header-summary {
            display: inline-flex;
            width: calc(100% - 20px);
            justify-content: space-between;
            align-items: center;
            vertical-align: middle;
        }
        .marine-group-list::-webkit-scrollbar {
            width: 6px;
        }
        .marine-group-list::-webkit-scrollbar-track {
            background: rgba(0, 0, 0, 0.15);
            border-radius: 3px;
        }
        .marine-group-list::-webkit-scrollbar-thumb {
            background: rgba(59, 130, 246, 0.3);
            border-radius: 3px;
        }
        .marine-group-list::-webkit-scrollbar-thumb:hover {
            background: rgba(59, 130, 246, 0.5);
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

        .alert-card.cancel-alert::before {
            background: var(--success);
        }

        .alert-card.out-of-region::before {
            background: var(--text-secondary);
        }

        .alert-card.marine-alert::before {
            background: var(--marine);
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

        .alert-card.cancel-alert .alert-type-badge {
            background: var(--success);
        }

        .alert-card.out-of-region .alert-type-badge {
            background: var(--text-secondary);
        }

        .alert-card.marine-alert .alert-type-badge {
            background: var(--marine);
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
            from { opacity: 0; }
            to { opacity: 1; }
        }

        @keyframes fadeIn {
            from { opacity: 0; }
            to { opacity: 1; }
        }

        /* 災害地図スタイル */
        .map-container {
            width: 100%;
            height: 350px;
            border-radius: 12px;
            border: 1px solid var(--card-border);
            overflow: hidden;
            position: relative;
            background: rgba(10, 7, 18, 0.4);
            margin-top: 0.5rem;
        }

        #offline-map {
            display: none;
            width: 100%;
            height: 100%;
            background: rgba(10, 7, 18, 0.6);
        }

        #leaflet-map {
            display: block;
            width: 100%;
            height: 100%;
        }

        /* Leaflet 暗色テーマカスタマイズ */
        .leaflet-container {
            background-color: var(--bg-secondary) !important;
            font-family: inherit;
        }

        .leaflet-popup-content-wrapper {
            background: var(--bg-secondary) !important;
            color: var(--text-primary) !important;
            border: 1px solid var(--card-border) !important;
            backdrop-filter: blur(12px);
            border-radius: 8px;
        }

        .leaflet-popup-tip {
            background: var(--bg-secondary) !important;
            border: 1px solid var(--card-border) !important;
        }

        /* デバッグモニタースタイル */
        .debug-grid {
            display: flex;
            flex-direction: column;
            gap: 0.6rem;
        }
        .debug-item {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 8px;
            padding: 0.5rem 0.8rem;
        }
        .debug-item-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0.5rem;
        }
        .debug-item-row > div {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 8px;
            padding: 0.5rem 0.8rem;
        }
        .debug-label {
            font-size: 0.7rem;
            color: var(--text-secondary);
            display: block;
            margin-bottom: 0.2rem;
            font-weight: 500;
        }
        .debug-value {
            font-size: 0.9rem;
            font-weight: 600;
            color: var(--text-primary);
        }
        .font-mono {
            font-family: 'Courier New', Courier, monospace;
        }

        /* アクション用アイコンボタン */
        .action-icon-btn {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid var(--card-border);
            color: var(--text-primary);
            border-radius: 8px;
            padding: 0.5rem;
            cursor: pointer;
            transition: all 0.2s ease;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            width: 38px;
            height: 38px;
        }

        .action-icon-btn:hover {
            background: rgba(255, 255, 255, 0.12);
            border-color: var(--card-border-hover);
            color: #fff;
            box-shadow: 0 0 8px rgba(255, 255, 255, 0.1);
        }

        .action-icon-btn:active {
            transform: scale(0.95);
        }

        .action-icon-btn.active {
            background: var(--accent);
            border-color: var(--accent);
            color: #fff;
            box-shadow: 0 2px 8px var(--accent-glow);
        }

        .action-icon-btn.danger-active {
            background: var(--danger);
            border-color: var(--danger);
            color: #fff;
            box-shadow: 0 2px 8px var(--danger-glow);
        }

        /* 履歴アイテムのレイアウト */
        .history-section {
            display: flex;
            flex-direction: column;
            gap: 0.8rem;
            max-height: 300px;
            overflow-y: auto;
            padding-right: 0.3rem;
        }

        /* スクロールバーのカスタマイズ */
        .history-section::-webkit-scrollbar {
            width: 6px;
        }
        .history-section::-webkit-scrollbar-track {
            background: rgba(255, 255, 255, 0.02);
            border-radius: 999px;
        }
        .history-section::-webkit-scrollbar-thumb {
            background: rgba(255, 255, 255, 0.15);
            border-radius: 999px;
        }
        .history-section::-webkit-scrollbar-thumb:hover {
            background: rgba(255, 255, 255, 0.3);
        }

        .history-item {
            background: rgba(255, 255, 255, 0.02);
            border: 1px solid var(--card-border);
            border-radius: 8px;
            padding: 0.8rem;
            display: flex;
            flex-direction: column;
            gap: 0.4rem;
            transition: all 0.2s ease;
        }

        .history-item:hover {
            border-color: var(--card-border-hover);
            background: rgba(255, 255, 255, 0.04);
        }

        .history-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .history-badge {
            font-size: 0.65rem;
            font-weight: 700;
            padding: 0.15rem 0.4rem;
            border-radius: 3px;
            color: #fff;
            background: var(--danger);
        }

        .history-badge.test {
            background: var(--warning);
        }

        .history-badge.out-of-region {
            background: var(--text-secondary);
        }

        .history-badge.cancel {
            background: var(--success);
        }

        .history-time {
            font-size: 0.7rem;
            color: var(--text-secondary);
        }

        .history-body {
            font-size: 0.85rem;
            color: var(--text-primary);
            line-height: 1.4;
        }

        .history-empty {
            font-size: 0.85rem;
            color: var(--text-secondary);
            text-align: center;
            padding: 2rem 0;
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
            <div style="display: flex; align-items: center; gap: 0.8rem;">
                <button id="btn-screen-off" class="action-icon-btn" onclick="toggleScreenOff()" title="液晶バックライト消灯">
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="width: 18px; height: 18px;"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"/><line x1="8" y1="21" x2="16" y2="21"/><line x1="12" y1="17" x2="12" y2="21"/></svg>
                </button>
                <div class="status-badge">
                    <div id="connection-dot" class="status-dot"></div>
                    <span id="connection-text">切断中</span>
                </div>
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

                <!-- 災害地図モニター -->
                <div class="card">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polygon points="3 6 9 3 15 6 21 3 21 18 15 21 9 18 3 21"/><line x1="9" y1="3" x2="9" y2="18"/><line x1="15" y1="6" x2="15" y2="21"/></svg>
                        災害地図モニター
                    </div>
                    <div class="map-container">
                        <div id="leaflet-map"></div>
                        <svg id="offline-map" viewBox="122 0 25 22">
                            <!-- Grid Lines -->
                            <line x1="125" y1="0" x2="125" y2="22" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="130" y1="0" x2="130" y2="22" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="135" y1="0" x2="135" y2="22" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="140" y1="0" x2="140" y2="22" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="145" y1="0" x2="145" y2="22" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            
                            <line x1="122" y1="2.5" x2="147" y2="2.5" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="5" x2="147" y2="5" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="7.5" x2="147" y2="7.5" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="10" x2="147" y2="10" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="12.5" x2="147" y2="12.5" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="15" x2="147" y2="15" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="17.5" x2="147" y2="17.5" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />
                            <line x1="122" y1="20" x2="147" y2="20" stroke="rgba(255,255,255,0.03)" stroke-width="0.05" />

                            <!-- Japan Outline Path -->
                            <!-- Hokkaido -->
                            <path d="M 141.7 0.6 L 145.8 2.7 L 143.2 4.0 L 140.7 4.2 L 140.1 4.6 L 140.4 2.7 Z" fill="rgba(139, 92, 246, 0.08)" stroke="rgba(139, 92, 246, 0.4)" stroke-width="0.1" />
                            <!-- Honshu -->
                            <path d="M 141.5 4.5 L 141.1 7.7 L 140.0 10.9 L 138.9 11.4 L 135.8 12.5 L 130.9 12.0 L 132.8 10.5 L 137.4 8.5 L 139.1 8.0 Z" fill="rgba(139, 92, 246, 0.08)" stroke="rgba(139, 92, 246, 0.4)" stroke-width="0.1" />
                            <!-- Shikoku -->
                            <path d="M 134.6 11.8 L 134.2 12.7 L 133.0 13.3 L 132.0 12.7 Z" fill="rgba(139, 92, 246, 0.08)" stroke="rgba(139, 92, 246, 0.4)" stroke-width="0.1" />
                            <!-- Kyushu -->
                            <path d="M 131.0 12.1 L 131.9 12.7 L 131.5 14.5 L 130.7 15.0 L 130.2 14.8 L 129.7 13.4 L 130.4 12.4 Z" fill="rgba(139, 92, 246, 0.08)" stroke="rgba(139, 92, 246, 0.4)" stroke-width="0.1" />
                            <!-- Okinawa -->
                            <path d="M 127.5 19.5 L 128.3 19.0 L 128.1 19.8 L 127.6 20.0 Z" fill="rgba(139, 92, 246, 0.08)" stroke="rgba(139, 92, 246, 0.4)" stroke-width="0.1" />

                            <!-- Dynamic overlays (pulsing red circles/orange dots) -->
                            <g id="svg-markers"></g>
                        </svg>
                    </div>
                </div>

                <!-- 警報履歴モニター -->
                <div class="card">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 8v4l3 3"/><circle cx="12" cy="12" r="10"/></svg>
                        警報履歴 (過去10件)
                    </div>
                    <div id="history-container" class="history-section">
                        <div class="history-empty">履歴はありません</div>
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
                                    <button id="btn-mute" type="button" class="action-icon-btn" onclick="toggleMute()" title="消音" style="margin-right: 0.2rem;">
                                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="width: 18px; height: 18px;"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><path d="M19.07 4.93a10 10 0 0 1 0 14.14M15.54 8.46a5 5 0 0 1 0 7.07"/></svg>
                                    </button>
                                    <input type="range" min="0" max="15" value="1" id="slider-vol" oninput="updateVolValue(this.value)" onchange="sendVolume(this.value)">
                                    <span class="slider-val" id="vol-value">1</span>
                                </div>
                            </div>
                            <div class="form-group">
                                <label for="select-region">自地域設定（警告フィルタ対象）</label>
                                <div class="input-row">
                                    <select id="select-region" class="form-control" style="background: rgba(255, 255, 255, 0.04); color: white; border: 1px solid var(--card-border); border-radius: 8px; padding: 0.6rem 0.8rem; font-family: inherit; font-size: 0.9rem; min-width: 150px;">
                                        <option value="0" style="background: var(--bg-secondary); color: white;">全国受信（フィルタ無効）</option>
                                        <option value="1" style="background: var(--bg-secondary); color: white;">北海道</option>
                                        <option value="2" style="background: var(--bg-secondary); color: white;">青森県</option>
                                        <option value="3" style="background: var(--bg-secondary); color: white;">岩手県</option>
                                        <option value="4" style="background: var(--bg-secondary); color: white;">宮城県</option>
                                        <option value="5" style="background: var(--bg-secondary); color: white;">秋田県</option>
                                        <option value="6" style="background: var(--bg-secondary); color: white;">山形県</option>
                                        <option value="7" style="background: var(--bg-secondary); color: white;">福島県</option>
                                        <option value="8" style="background: var(--bg-secondary); color: white;">茨城県</option>
                                        <option value="9" style="background: var(--bg-secondary); color: white;">栃木県</option>
                                        <option value="10" style="background: var(--bg-secondary); color: white;">群馬県</option>
                                        <option value="11" style="background: var(--bg-secondary); color: white;">埼玉県</option>
                                        <option value="12" style="background: var(--bg-secondary); color: white;">千葉県</option>
                                        <option value="13" style="background: var(--bg-secondary); color: white;">東京都</option>
                                        <option value="14" style="background: var(--bg-secondary); color: white;">神奈川県</option>
                                        <option value="15" style="background: var(--bg-secondary); color: white;">新潟県</option>
                                        <option value="16" style="background: var(--bg-secondary); color: white;">富山県</option>
                                        <option value="17" style="background: var(--bg-secondary); color: white;">石川県</option>
                                        <option value="18" style="background: var(--bg-secondary); color: white;">福井県</option>
                                        <option value="19" style="background: var(--bg-secondary); color: white;">山梨県</option>
                                        <option value="20" style="background: var(--bg-secondary); color: white;">長野県</option>
                                        <option value="21" style="background: var(--bg-secondary); color: white;">岐阜県</option>
                                        <option value="22" style="background: var(--bg-secondary); color: white;">静岡県</option>
                                        <option value="23" style="background: var(--bg-secondary); color: white;">愛知県</option>
                                        <option value="24" style="background: var(--bg-secondary); color: white;">三重県</option>
                                        <option value="25" style="background: var(--bg-secondary); color: white;">滋賀県</option>
                                        <option value="26" style="background: var(--bg-secondary); color: white;">京都府</option>
                                        <option value="27" style="background: var(--bg-secondary); color: white;">大阪府</option>
                                        <option value="28" style="background: var(--bg-secondary); color: white;">兵庫県</option>
                                        <option value="29" style="background: var(--bg-secondary); color: white;">奈良県</option>
                                        <option value="30" style="background: var(--bg-secondary); color: white;">和歌山県</option>
                                        <option value="31" style="background: var(--bg-secondary); color: white;">鳥取県</option>
                                        <option value="32" style="background: var(--bg-secondary); color: white;">島根県</option>
                                        <option value="33" style="background: var(--bg-secondary); color: white;">岡山県</option>
                                        <option value="34" style="background: var(--bg-secondary); color: white;">広島県</option>
                                        <option value="35" style="background: var(--bg-secondary); color: white;">山口県</option>
                                        <option value="36" style="background: var(--bg-secondary); color: white;">徳島県</option>
                                        <option value="37" style="background: var(--bg-secondary); color: white;">香川県</option>
                                        <option value="38" style="background: var(--bg-secondary); color: white;">愛媛県</option>
                                        <option value="39" style="background: var(--bg-secondary); color: white;">高知県</option>
                                        <option value="40" style="background: var(--bg-secondary); color: white;">福岡県</option>
                                        <option value="41" style="background: var(--bg-secondary); color: white;">佐賀県</option>
                                        <option value="42" style="background: var(--bg-secondary); color: white;">長崎県</option>
                                        <option value="43" style="background: var(--bg-secondary); color: white;">熊本県</option>
                                        <option value="44" style="background: var(--bg-secondary); color: white;">大分県</option>
                                        <option value="45" style="background: var(--bg-secondary); color: white;">宮崎県</option>
                                        <option value="46" style="background: var(--bg-secondary); color: white;">鹿児島県</option>
                                        <option value="47" style="background: var(--bg-secondary); color: white;">沖縄県</option>
                                    </select>
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
                        <button class="btn btn-secondary" onclick="triggerMock('jalert')">模擬 Jアラート</button>
                        <button class="btn btn-secondary" onclick="triggerMock('lalert')">模擬 Lアラート</button>
                    </div>
                    <div class="btn-grid" style="grid-template-columns: 1fr;">
                        <button class="btn btn-secondary" onclick="triggerMock('marine')">模擬 海上警報(複数)</button>
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

                <!-- GPS & みちびき デバッグモニター -->
                <div class="card" style="margin-top: 1.5rem;">
                    <div class="card-title">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="2" y="2" width="20" height="20" rx="2" ry="2"/><rect x="6" y="6" width="12" height="12"/></svg>
                        GPS &amp; みちびき デバッグモニター
                    </div>
                    <div class="debug-grid">
                        <div class="debug-item">
                            <span class="debug-label">最終 GGA センテンス</span>
                            <span class="debug-value font-mono" id="debug-gga" style="font-size:0.75rem; word-break:break-all;">--</span>
                        </div>
                        <div class="debug-item-row">
                            <div>
                                <span class="debug-label">SFRBX 受信数</span>
                                <span class="debug-value" id="debug-sfrbx">0</span>
                            </div>
                            <div>
                                <span class="debug-label">最終受信からの経過秒</span>
                                <span class="debug-value" id="debug-l1s-elapsed">--</span>
                            </div>
                        </div>
                        <div class="debug-item-row">
                            <div>
                                <span class="debug-label">MT43 (防災) 受信数</span>
                                <span class="debug-value" id="debug-mt43">0</span>
                            </div>
                            <div>
                                <span class="debug-label">MT44 (Jアラ) 受信数</span>
                                <span class="debug-value" id="debug-mt44">0</span>
                            </div>
                        </div>
                        <div class="debug-item">
                            <span class="debug-label">最終みちびき生 L1S パケット (HEX)</span>
                            <span class="debug-value font-mono" id="debug-l1s-hex" style="font-size:0.7rem; word-break:break-all; color: var(--info);">--</span>
                        </div>
                    </div>
                </div>

            </div>
        </div>
    </div>

    <script>
        function getEllipsePolygon(lat, lon, semiMajorKm, semiMinorKm, azimuthDeg, numPoints = 64) {
            const coords = [];
            const R = 6378.137; // 地球の半径 (km)
            const azimuthRad = (azimuthDeg * Math.PI) / 180;
            for (let i = 0; i < numPoints; i++) {
                const phi = (i * 2 * Math.PI) / numPoints;
                const x_local = semiMajorKm * Math.cos(phi);
                const y_local = semiMinorKm * Math.sin(phi);
                // 角度（時計回り）で回転
                const y_rot = x_local * Math.cos(azimuthRad) - y_local * Math.sin(azimuthRad);
                const x_rot = x_local * Math.sin(azimuthRad) + y_local * Math.cos(azimuthRad);
                const dLat = (y_rot / R) * (180 / Math.PI);
                const dLon = (x_rot / (R * Math.cos((lat * Math.PI) / 180))) * (180 / Math.PI);
                coords.push([lat + dLat, lon + dLon]);
            }
            return coords;
        }

        let isAPMode = false;
        let lastIp = "";
        let currentMute = false;
        let currentScreenOff = false;

        let map = null;
        let leafletLoaded = false;
        let mapMarkers = [];

        const PREFECTURE_COORDINATES = {
            1: { name: "北海道", lat: 43.46, lon: 142.82 },
            2: { name: "青森県", lat: 40.73, lon: 140.83 },
            3: { name: "岩手県", lat: 39.56, lon: 141.33 },
            4: { name: "宮城県", lat: 38.38, lon: 140.82 },
            5: { name: "秋田県", lat: 39.75, lon: 140.40 },
            6: { name: "山形県", lat: 38.45, lon: 140.12 },
            7: { name: "福島県", lat: 37.38, lon: 140.28 },
            8: { name: "茨城県", lat: 36.32, lon: 140.27 },
            9: { name: "栃木県", lat: 36.65, lon: 140.04 },
            10: { name: "群馬県", lat: 36.48, lon: 138.93 },
            11: { name: "埼玉県", lat: 35.98, lon: 139.37 },
            12: { name: "千葉県", lat: 35.51, lon: 140.23 },
            13: { name: "東京都", lat: 35.68, lon: 139.40 },
            14: { name: "神奈川県", lat: 35.43, lon: 139.25 },
            15: { name: "新潟県", lat: 37.49, lon: 138.90 },
            16: { name: "富山県", lat: 36.65, lon: 137.29 },
            17: { name: "石川県", lat: 36.76, lon: 136.85 },
            18: { name: "福井県", lat: 35.91, lon: 136.19 },
            19: { name: "山梨県", lat: 35.61, lon: 138.60 },
            20: { name: "長野県", lat: 36.10, lon: 137.97 },
            21: { name: "岐阜県", lat: 35.75, lon: 137.04 },
            22: { name: "静岡県", lat: 34.98, lon: 138.38 },
            23: { name: "愛知県", lat: 35.00, lon: 136.94 },
            24: { name: "三重県", lat: 34.42, lon: 136.45 },
            25: { name: "滋賀県", lat: 35.20, lon: 136.14 },
            26: { name: "京都府", lat: 35.25, lon: 135.45 },
            27: { name: "大阪府", lat: 34.60, lon: 135.53 },
            28: { name: "兵庫県", lat: 35.03, lon: 134.80 },
            29: { name: "奈良県", lat: 34.28, lon: 135.86 },
            30: { name: "和歌山県", lat: 33.90, lon: 135.45 },
            31: { name: "鳥取県", lat: 35.39, lon: 133.85 },
            32: { name: "島根県", lat: 34.89, lon: 132.63 },
            33: { name: "岡山県", lat: 34.86, lon: 133.78 },
            34: { name: "広島県", lat: 34.54, lon: 132.78 },
            35: { name: "山口県", lat: 34.12, lon: 131.57 },
            36: { name: "徳島県", lat: 33.91, lon: 134.29 },
            37: { name: "香川県", lat: 34.26, lon: 133.95 },
            38: { name: "愛媛県", lat: 33.68, lon: 132.86 },
            39: { name: "高知県", lat: 33.45, lon: 133.30 },
            40: { name: "福岡県", lat: 33.60, lon: 130.66 },
            41: { name: "佐賀県", lat: 33.28, lon: 130.06 },
            42: { name: "長崎県", lat: 32.88, lon: 129.98 },
            43: { name: "熊本県", lat: 32.50, lon: 130.80 },
            44: { name: "大分県", lat: 33.17, lon: 131.42 },
            45: { name: "宮崎県", lat: 32.17, lon: 131.35 },
            46: { name: "鹿児島県", lat: 31.32, lon: 130.64 },
            47: { name: "沖縄県", lat: 26.30, lon: 127.80 }
        };

        const TSUNAMI_REGION_COORDINATES = {
            200: { lat: 41.5, lon: 141.8 },
            210: { lat: 40.2, lon: 142.1 },
            220: { lat: 38.3, lon: 141.7 },
            230: { lat: 37.3, lon: 141.2 },
            240: { lat: 36.3, lon: 140.8 },
            250: { lat: 35.1, lon: 140.3 },
            300: { lat: 35.0, lon: 139.5 },
            310: { lat: 34.5, lon: 138.8 },
            320: { lat: 34.0, lon: 137.5 }
        };

        const MARINE_REGION_COORDINATES = {
            1000: { lat: 45.0, lon: 141.0, radius: 200000 }, // 日本海北部オホーツク南部
            1010: { lat: 48.0, lon: 145.0, radius: 150000 }, // サハリン東方海上
            1020: { lat: 48.0, lon: 141.0, radius: 150000 }, // サハリン西方海上
            1030: { lat: 44.5, lon: 144.5, radius: 80000 }, // 網走沖
            1040: { lat: 45.8, lon: 142.0, radius: 50000 }, // 宗谷海峡
            1050: { lat: 43.5, lon: 140.0, radius: 100000 }, // 北海道西方海上
            1100: { lat: 41.5, lon: 145.0, radius: 150000 }, // 北海道南方東方海上
            1110: { lat: 43.0, lon: 147.0, radius: 150000 }, // 北海道東方海上
            1120: { lat: 42.5, lon: 144.0, radius: 80000 }, // 釧路沖
            1130: { lat: 41.8, lon: 142.5, radius: 60000 }, // 日高沖
            1140: { lat: 41.5, lon: 140.7, radius: 40000 }, // 津軽海峡
            1150: { lat: 41.5, lon: 139.5, radius: 60000 }, // 檜山津軽沖
            2000: { lat: 39.0, lon: 143.0, radius: 150000 }, // 三陸沖
            2010: { lat: 39.0, lon: 145.0, radius: 120000 }, // 三陸沖東部
            2020: { lat: 39.5, lon: 142.5, radius: 80000 }, // 三陸沖西部
            3000: { lat: 34.5, lon: 141.0, radius: 150000 }, // 関東海域
            3010: { lat: 35.5, lon: 141.5, radius: 80000 }, // 関東海域北部
            3020: { lat: 33.5, lon: 140.5, radius: 100000 }, // 関東海域南部
            3100: { lat: 40.0, lon: 135.0, radius: 250000 }, // 日本海中部
            3110: { lat: 41.0, lon: 132.0, radius: 120000 }, // 沿海州南部沖
            3120: { lat: 40.0, lon: 139.0, radius: 80000 }, // 秋田沖
            3130: { lat: 38.3, lon: 137.5, radius: 80000 }, // 佐渡沖
            3140: { lat: 37.8, lon: 136.5, radius: 60000 }, // 能登沖
            3200: { lat: 33.5, lon: 137.5, radius: 150000 }, // 東海海域
            3210: { lat: 34.0, lon: 138.5, radius: 80000 }, // 東海海域東部
            3220: { lat: 33.5, lon: 137.0, radius: 80000 }, // 東海海域西部
            3230: { lat: 31.5, lon: 137.5, radius: 120000 }, // 東海海域南部
            4000: { lat: 32.5, lon: 134.0, radius: 150000 }, // 四国沖瀬戸内海
            4010: { lat: 34.3, lon: 133.5, radius: 80000 }, // 瀬戸内海
            4020: { lat: 32.5, lon: 133.5, radius: 80000 }, // 四国沖北部
            4030: { lat: 30.5, lon: 133.5, radius: 120000 }, // 四国沖南部
            4100: { lat: 36.5, lon: 131.0, radius: 150000 }, // 日本海西部
            4110: { lat: 38.0, lon: 131.0, radius: 120000 }, // 日本海北西部
            4120: { lat: 36.3, lon: 135.0, radius: 100000 }, // 山陰沖東部若狭湾付近
            4130: { lat: 35.8, lon: 132.0, radius: 80000 }, // 山陰沖西部
            5000: { lat: 34.0, lon: 129.5, radius: 60000 }, // 対馬海峡
            5100: { lat: 32.0, lon: 128.0, radius: 120000 }, // 九州西方海上
            5110: { lat: 33.5, lon: 125.0, radius: 100000 }, // 済州島西海上
            5120: { lat: 32.5, lon: 128.5, radius: 80000 }, // 長崎西海上
            5130: { lat: 31.0, lon: 127.0, radius: 100000 }, // 女島南西海上
            5200: { lat: 31.0, lon: 132.0, radius: 150000 }, // 九州南方海上・日向灘
            5210: { lat: 32.0, lon: 132.0, radius: 80000 }, // 日向灘
            5220: { lat: 30.5, lon: 130.5, radius: 80000 }, // 鹿児島海域
            5230: { lat: 28.5, lon: 129.5, radius: 100000 }, // 奄美海域
            6000: { lat: 26.0, lon: 126.0, radius: 150000 }, // 沖縄海域
            6010: { lat: 27.0, lon: 124.0, radius: 120000 }, // 東シナ海南部
            6020: { lat: 26.0, lon: 129.0, radius: 120000 }, // 沖縄東方海上
            6030: { lat: 24.0, lon: 126.0, radius: 120000 }, // 沖縄南方海上
            10000: { lat: 35.0, lon: 145.0, radius: 150000 } // その他海域
        };

        const VOLCANO_COORDINATES = {
            104: { lat: 42.54, lon: 140.84 },
            201: { lat: 40.65, lon: 140.35 },
            301: { lat: 37.64, lon: 139.70 },
            501: { lat: 35.36, lon: 138.73 },
            504: { lat: 35.23, lon: 139.02 },
            506: { lat: 36.40, lon: 138.52 },
            701: { lat: 34.72, lon: 137.60 },
            902: { lat: 32.88, lon: 131.10 },
            905: { lat: 31.59, lon: 130.65 }
        };

        // Leaflet CDNからリソースをロード
        function loadMapAssets() {
            const link = document.createElement('link');
            link.rel = 'stylesheet';
            link.href = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
            document.head.appendChild(link);

            const script = document.createElement('script');
            script.src = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
            script.onload = () => {
                initMap();
            };
            script.onerror = () => {
                initMapFallback();
            };
            document.head.appendChild(script);

            // タイムアウト監視 (3秒)
            setTimeout(() => {
                if (typeof L === 'undefined') {
                    console.warn('Leaflet load timed out. Falling back to offline SVG map.');
                    initMapFallback();
                }
            }, 3000);
        }

        function initMap() {
            if (typeof L !== 'undefined') {
                leafletLoaded = true;
                document.getElementById('leaflet-map').style.display = 'block';
                document.getElementById('offline-map').style.display = 'none';

                map = L.map('leaflet-map', {
                    zoomControl: false,
                    attributionControl: false
                }).setView([36.2048, 138.2529], 5);

                L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
                    maxZoom: 10,
                    minZoom: 3
                }).addTo(map);

                L.control.zoom({ position: 'bottomright' }).addTo(map);
            } else {
                initMapFallback();
            }
        }

        function initMapFallback() {
            leafletLoaded = false;
            document.getElementById('leaflet-map').style.display = 'none';
            document.getElementById('offline-map').style.display = 'block';
        }

        function updateMap(alerts) {
            if (leafletLoaded && map) {
                mapMarkers.forEach(m => map.removeLayer(m));
                mapMarkers = [];
            }
            
            const svgGroup = document.getElementById('svg-markers');
            if (svgGroup) {
                svgGroup.innerHTML = '';
            }

            if (!alerts || alerts.length === 0) return;

            alerts.forEach(alert => {
                let lat = alert.lat || 0;
                let lon = alert.lon || 0;
                let isExact = (lat !== 0 && lon !== 0);
                
                let locations = [];
                let color = '#ef4444';

                if (isExact) {
                    if (alert.elMajor && alert.elMajor > 0) {
                        locations.push({ lat: lat, lon: lon, color: '#ef4444', label: '災害地域(中心)', isEllipse: true });
                    } else {
                        locations.push({ lat: lat, lon: lon, color: '#ef4444', label: '震源地' });
                    }
                } else {
                    if (alert.cat === 1) { // EEW
                        let approx = getApproxEpicenterCoords(alert.code);
                        if (approx) {
                            locations.push({ lat: approx.lat, lon: approx.lon, color: '#ef4444', label: '震源(推定)' });
                        }
                    } else if (alert.cat === 3 || alert.cat === 10) { // 震度 or 気象
                        if (alert.code >= 1 && alert.code <= 47) {
                            const coords = PREFECTURE_COORDINATES[alert.code];
                            locations.push({ lat: coords.lat, lon: coords.lon, color: '#f59e0b', label: coords.name });
                        }
                    } else if (alert.cat === 5 || alert.cat === 6) { // 津波
                        const coords = TSUNAMI_REGION_COORDINATES[alert.code] || { lat: 36.0, lon: 140.0 };
                        locations.push({ lat: coords.lat, lon: coords.lon, color: '#06b6d4', label: '津波対象地域' });
                    } else if (alert.cat === 14) { // 海上警報
                        const coords = MARINE_REGION_COORDINATES[alert.code] || { lat: 35.0, lon: 143.0 };
                        locations.push({ lat: coords.lat, lon: coords.lon, color: '#3b82f6', label: '対象海域', radius: coords.radius });
                    } else if (alert.cat === 8 || alert.cat === 9) { // 火山
                        const coords = VOLCANO_COORDINATES[alert.code] || { lat: 35.36, lon: 138.73 };
                        locations.push({ lat: coords.lat, lon: coords.lon, color: '#ec4899', label: '火山' });
                    } else if (alert.cat === 44) { // Jアラート
                        try {
                            const mask = BigInt("0x" + alert.prefMask);
                            for (let i = 1; i <= 47; i++) {
                                const bit = BigInt(1) << BigInt(64 - i);
                                if ((mask & bit) !== 0n) {
                                    const coords = PREFECTURE_COORDINATES[i];
                                    locations.push({ lat: coords.lat, lon: coords.lon, color: '#f59e0b', label: coords.name });
                                }
                            }
                        } catch (e) {
                            console.error("Failed to parse prefMask:", e);
                        }
                    }
                }

                locations.forEach(loc => {
                    if (leafletLoaded && map) {
                        if (loc.isEllipse) {
                            const coords = getEllipsePolygon(loc.lat, loc.lon, alert.elMajor, alert.elMinor, alert.elAzimuth);
                            const markerPolygon = L.polygon(coords, {
                                color: loc.color,
                                fillColor: loc.color,
                                fillOpacity: 0.2,
                                weight: 2
                            }).addTo(map);
                            markerPolygon.bindPopup(`<b>${alert.text}</b>`);
                            mapMarkers.push(markerPolygon);
                        } else {
                            const radius = loc.radius || (alert.cat === 44 ? 30000 : 50000);
                            const markerCircle = L.circle([loc.lat, loc.lon], {
                                color: loc.color,
                                fillColor: loc.color,
                                fillOpacity: 0.35,
                                radius: radius
                            }).addTo(map);
                            markerCircle.bindPopup(`<b>${alert.text}</b>`);
                            mapMarkers.push(markerCircle);
                        }
                    }
                    
                    if (svgGroup) {
                        const x = loc.lon;
                        const y = 46 - loc.lat;
                        
                        if (x >= 122 && x <= 147 && y >= 0 && y <= 22) {
                            const pulseElement = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
                            pulseElement.setAttribute('cx', x);
                            pulseElement.setAttribute('cy', y);
                            
                            let rVal = 0.6;
                            if (loc.isEllipse) {
                                rVal = Math.min(Math.max(alert.elMajor / 111.0, 0.2), 3.0);
                            } else if (loc.radius) {
                                rVal = loc.radius / 111000.0;
                            } else if (alert.cat === 5) {
                                rVal = 0.4;
                            }
                            
                            pulseElement.setAttribute('r', rVal);
                            pulseElement.setAttribute('fill', hexToRgba(loc.color, 0.4));
                            pulseElement.setAttribute('stroke', loc.color);
                            pulseElement.setAttribute('stroke-width', '0.08');
                            
                            const animateR = document.createElementNS('http://www.w3.org/2000/svg', 'animate');
                            animateR.setAttribute('attributeName', 'r');
                            animateR.setAttribute('values', `${rVal * 0.2};${rVal * 1.3};${rVal * 0.2}`);
                            animateR.setAttribute('dur', '2s');
                            animateR.setAttribute('repeatCount', 'indefinite');
                            pulseElement.appendChild(animateR);
                            
                            svgGroup.appendChild(pulseElement);
                            
                            const centerElement = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
                            centerElement.setAttribute('cx', x);
                            centerElement.setAttribute('cy', y);
                            centerElement.setAttribute('r', '0.15');
                            centerElement.setAttribute('fill', loc.color);
                            svgGroup.appendChild(centerElement);
                        }
                    }
                });
            });
        }

        function hexToRgba(hex, alpha) {
            const r = parseInt(hex.slice(1, 3), 16);
            const g = parseInt(hex.slice(3, 5), 16);
            const b = parseInt(hex.slice(5, 7), 16);
            return `rgba(${r}, ${g}, ${b}, ${alpha})`;
        }

        function getApproxEpicenterCoords(code) {
            if (code === 11) return { lat: 43.5, lon: 142.5 };
            if (code === 12) return { lat: 39.5, lon: 141.0 };
            if (code === 13) return { lat: 37.0, lon: 137.5 };
            if (code === 14) return { lat: 36.0, lon: 139.5 };
            if (code === 15) return { lat: 27.0, lon: 142.0 };
            if (code === 16) return { lat: 35.0, lon: 137.5 };
            if (code === 17) return { lat: 35.0, lon: 135.5 };
            if (code === 18) return { lat: 35.0, lon: 133.0 };
            if (code === 19) return { lat: 33.5, lon: 133.5 };
            if (code === 20) return { lat: 32.5, lon: 131.0 };
            if (code === 21) return { lat: 26.5, lon: 128.0 };
            
            if (code >= 100 && code < 200) return { lat: 43.5, lon: 142.5 };
            if (code >= 200 && code < 210) return { lat: 40.5, lon: 141.0 };
            if (code >= 210 && code < 220) return { lat: 39.5, lon: 141.5 };
            if (code >= 220 && code < 230) return { lat: 38.3, lon: 141.0 };
            if (code >= 300 && code < 400) return { lat: 35.5, lon: 139.5 };
            if (code >= 500 && code < 600) return { lat: 35.0, lon: 135.5 };
            return { lat: 36.0, lon: 138.0 };
        }

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

                // ミュートボタンの更新
                currentMute = data.mute;
                const muteBtn = document.getElementById('btn-mute');
                if (muteBtn) {
                    if (currentMute) {
                        muteBtn.classList.add('danger-active');
                        muteBtn.title = "消音中 (クリックでミュート解除)";
                        muteBtn.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="width: 18px; height: 18px;"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><line x1="23" y1="9" x2="17" y2="15"/><line x1="17" y1="9" x2="23" y2="15"/></svg>`;
                    } else {
                        muteBtn.classList.remove('danger-active');
                        muteBtn.title = "音声出力中 (クリックで消音)";
                        muteBtn.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="width: 18px; height: 18px;"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><path d="M19.07 4.93a10 10 0 0 1 0 14.14M15.54 8.46a5 5 0 0 1 0 7.07"/></svg>`;
                    }
                }

                // 画面消灯ボタンの更新
                currentScreenOff = data.screenOff;
                const screenOffBtn = document.getElementById('btn-screen-off');
                if (screenOffBtn) {
                    if (currentScreenOff) {
                        screenOffBtn.classList.add('active');
                        screenOffBtn.title = "画面消灯有効 (クリックで常時点灯)";
                    } else {
                        screenOffBtn.classList.remove('active');
                        screenOffBtn.title = "常時点灯 (クリックで画面消灯)";
                    }
                }

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

                // デバッグ情報の更新
                document.getElementById('debug-gga').innerText = data.gpsGga || "--";
                document.getElementById('debug-sfrbx').innerText = data.qzssSfrbxCount || 0;
                document.getElementById('debug-mt43').innerText = data.qzssMt43Count || 0;
                document.getElementById('debug-mt44').innerText = data.qzssMt44Count || 0;
                document.getElementById('debug-l1s-hex').innerText = data.lastL1sHex || "--";

                const elapsed = data.sinceLastL1s;
                const elapsedEl = document.getElementById('debug-l1s-elapsed');
                if (elapsed === 99999) {
                    elapsedEl.innerText = "受信なし";
                    elapsedEl.style.color = "var(--text-secondary)";
                } else {
                    elapsedEl.innerText = `${elapsed}秒前`;
                    if (elapsed < 10) {
                        elapsedEl.style.color = "var(--success)";
                    } else if (elapsed < 30) {
                        elapsedEl.style.color = "var(--warning)";
                    } else {
                        elapsedEl.style.color = "var(--danger)";
                    }
                }

                // フォームの値を初回のみ設定
                if (!document.activeElement || (document.activeElement.tagName !== 'INPUT' && document.activeElement.tagName !== 'SELECT')) {
                    if (document.getElementById('input-freq').value === "") {
                        document.getElementById('input-freq').value = data.freq.toFixed(1);
                    }
                    const selectRegion = document.getElementById('select-region');
                    if (selectRegion && selectRegion.value === "") {
                        selectRegion.value = data.region;
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
                    const currentAlertsSignature = data.alerts.map(a => a.text + "|" + a.isTest + "|" + a.outOfRegion + "|" + a.cat).join("||");
                    if (alertsContainer.getAttribute('data-signature') !== currentAlertsSignature) {
                        alertsContainer.setAttribute('data-signature', currentAlertsSignature);
                        let html = '';
                        let hasRenderedMarineGroup = false;
                        const marineAlerts = data.alerts.filter(a => a.cat === 14);
                        
                        data.alerts.forEach((alert, idx) => {
                            if (alert.cat === 14) {
                                if (hasRenderedMarineGroup) {
                                    return; // すでにグループ描画済みの場合はスキップ
                                }
                                hasRenderedMarineGroup = true;
                                
                                let cardClass = 'alert-card marine-alert';
                                let badgeText = '海上警報';
                                
                                if (marineAlerts.length === 1) {
                                    // 1件だけなら通常通り表示
                                    html += `
                                        <div class="${cardClass}" id="alert-card-${idx}">
                                            <div class="alert-header">
                                                <span class="alert-type-badge">${badgeText}</span>
                                                <span class="alert-expiry" id="alert-expiry-${idx}"></span>
                                            </div>
                                            <div class="alert-body">${alert.text}</div>
                                        </div>
                                    `;
                                } else {
                                    // 複数ある場合はアコーディオン(details)で折りたたむ
                                    html += `
                                        <div class="${cardClass}" id="alert-card-${idx}">
                                            <details open class="marine-details">
                                                <summary class="marine-summary">
                                                    <div class="alert-header-summary">
                                                        <span class="alert-type-badge">${badgeText} (${marineAlerts.length}海域)</span>
                                                        <span class="alert-expiry" id="alert-expiry-marine-group-header"></span>
                                                    </div>
                                                </summary>
                                                <div class="marine-group-list" style="margin-top: 10px; max-height: 180px; overflow-y: auto; border-top: 1px solid rgba(255, 255, 255, 0.15); padding-top: 8px;">
                                                    ${marineAlerts.map(ma => {
                                                        const origIdx = data.alerts.findIndex(a => a.text === ma.text && a.cat === ma.cat);
                                                        return `
                                                            <div class="marine-sub-item" style="padding: 6px 10px; border-bottom: 1px solid rgba(255, 255, 255, 0.08); display: flex; justify-content: space-between; font-size: 0.9em; align-items: center; transition: background 0.2s;" onmouseover="this.style.background='rgba(59,130,246,0.1)'" onmouseout="this.style.background='transparent'">
                                                                <span class="marine-sub-text" style="color: var(--text-primary); text-shadow: 0 0 10px rgba(255,255,255,0.1);">${ma.text}</span>
                                                                <span class="marine-sub-expiry alert-expiry" style="opacity: 0.85; font-size: 0.85em; white-space: nowrap; color: var(--marine);" id="alert-expiry-${origIdx}"></span>
                                                            </div>
                                                        `;
                                                    }).join('')}
                                                </div>
                                            </details>
                                        </div>
                                    `;
                                }
                            } else {
                                const isTest = alert.isTest;
                                const isOutOfRegion = alert.outOfRegion;
                                const isCancel = alert.text.includes("解除") || alert.text.includes("取消") || alert.text.includes("終了");
                                let cardClass = 'alert-card';
                                let badgeText = '災害警報';
                                
                                if (isCancel) {
                                    cardClass = 'alert-card cancel-alert';
                                    badgeText = '解除/情報';
                                } else if (isOutOfRegion) {
                                    cardClass = 'alert-card out-of-region';
                                    badgeText = '他地域・対象外';
                                } else if (isTest) {
                                    cardClass = 'alert-card test-alert';
                                    badgeText = '訓練/試験';
                                }
                                
                                html += `
                                    <div class="${cardClass}" id="alert-card-${idx}">
                                        <div class="alert-header">
                                            <span class="alert-type-badge">${badgeText}</span>
                                            <span class="alert-expiry" id="alert-expiry-${idx}"></span>
                                        </div>
                                        <div class="alert-body">${alert.text}</div>
                                    </div>
                                `;
                            }
                        });
                        alertsContainer.innerHTML = html;
                    }
                    
                    // 有効期限の数値のみをインプレースで更新（アニメーションの再発火を防止）
                    let maxMarineRemaining = 0;
                    data.alerts.forEach((alert, idx) => {
                        if (alert.cat === 14) {
                            if (alert.remaining > maxMarineRemaining) {
                                maxMarineRemaining = alert.remaining;
                            }
                        }
                        const expiryEl = document.getElementById(`alert-expiry-${idx}`);
                        if (expiryEl) {
                            let expiryText = "";
                            if (alert.remaining > 0) {
                                expiryText = `有効期限: 約${alert.remaining}秒`;
                            } else if (alert.remaining === 0) {
                                expiryText = "期限切れ間近";
                            }
                            
                            // サブアイテム内の表示用に秒数表示のみにする
                            const isSubItem = expiryEl.classList.contains('marine-sub-expiry');
                            if (isSubItem) {
                                expiryEl.innerText = alert.remaining > 0 ? `約${alert.remaining}秒` : "間近";
                            } else {
                                expiryEl.innerText = expiryText;
                            }
                        }
                    });
                    
                    // 海上警報グループヘッダーの有効期限を更新
                    const marineHeaderExpiryEl = document.getElementById('alert-expiry-marine-group-header');
                    if (marineHeaderExpiryEl) {
                        marineHeaderExpiryEl.innerText = maxMarineRemaining > 0 ? `有効期限: 約${maxMarineRemaining}秒` : "期限切れ間近";
                    }
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
                updateMap(data.alerts || []);

                // 警報履歴の更新
                const historyContainer = document.getElementById('history-container');
                if (historyContainer) {
                    if (data.history && data.history.length > 0) {
                        const currentHistorySignature = data.history.map(h => h.text + "|" + h.time + "|" + h.isTest + "|" + h.outOfRegion).join("||");
                        if (historyContainer.getAttribute('data-signature') !== currentHistorySignature) {
                            historyContainer.setAttribute('data-signature', currentHistorySignature);
                            let html = '';
                            data.history.forEach(hist => {
                                const isTest = hist.isTest;
                                const isOutOfRegion = hist.outOfRegion;
                                const isCancel = hist.text.includes("解除") || hist.text.includes("取消") || hist.text.includes("終了");
                                let badgeClass = 'history-badge';
                                let badgeText = '災害警報';
                                
                                if (isCancel) {
                                    badgeClass = 'history-badge cancel';
                                    badgeText = '解除';
                                } else if (isOutOfRegion) {
                                    badgeClass = 'history-badge out-of-region';
                                    badgeText = '他地域';
                                } else if (isTest) {
                                    badgeClass = 'history-badge test';
                                    badgeText = '訓練';
                                }
                                
                                html += `
                                    <div class="history-item">
                                        <div class="history-header">
                                            <span class="${badgeClass}">${badgeText}</span>
                                            <span class="history-time">${hist.time}</span>
                                        </div>
                                        <div class="history-body">${hist.text}</div>
                                    </div>
                                `;
                            });
                            historyContainer.innerHTML = html;
                        }
                    } else {
                        if (historyContainer.getAttribute('data-signature') !== "empty") {
                            historyContainer.setAttribute('data-signature', "empty");
                            historyContainer.innerHTML = `<div class="history-empty">履歴はありません</div>`;
                        }
                    }
                }
            } catch (err) {
                console.error(err);
                document.getElementById('connection-dot').classList.remove('online');
                document.getElementById('connection-text').innerText = "切断中 (再接続試行...)";
            }
        }

        // 消音切り替え
        async function toggleMute() {
            await sendSettings({ mute: currentMute ? 0 : 1 });
        }

        // 画面消灯切り替え
        async function toggleScreenOff() {
            await sendSettings({ screenOff: currentScreenOff ? 0 : 1 });
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
            const selectEl = document.getElementById('select-region');
            if (selectEl) {
                const reg = parseInt(selectEl.value);
                if (reg >= 0 && reg <= 99) {
                    await sendSettings({ region: reg });
                }
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
        loadMapAssets();
        fetchStatus();
        setInterval(fetchStatus, 1500);
    </script>
</body>
</html>
)rawhtml";

#endif // WEBUI_HTML_H
