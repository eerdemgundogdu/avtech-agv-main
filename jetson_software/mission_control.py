#!/usr/bin/env python3
# AVTECH AGV - Gorev Senaryosu (Mission Control)
# Robotun haritada veya cizgi uzerinde ne yapacagini, engellerde veya hedeflerde 
# nasil tepki verecegini belirten senaryo motoru. Yari-otonom ve otonom karar mekanizmasi.

import time
import enum
import json


class MissionState(enum.Enum):
    """Otonom gorev durumlari."""
    IDLE             = "IDLE"
    LINE_FOLLOWING   = "LINE_FOLLOWING"
    QR_DETECTED      = "QR_DETECTED"
    APPROACHING_QR   = "APPROACHING_QR"
    STOPPING         = "STOPPING"
    EXECUTING_AL     = "EXECUTING_AL"
    EXECUTING_BIRAK  = "EXECUTING_BIRAK"
    WAITING_ACTION   = "WAITING_ACTION"
    RESUMING         = "RESUMING"
    FINISHED         = "FINISHED"
    ERROR            = "ERROR"


class MissionControl:
    """Otonom gorev karar mekanizmasi (State Machine).

    Her telemetri dongusunde (100 Hz) ``update()`` metodu cagrilir.
    Non-blocking tasarim: hicbir yerde ``time.sleep()`` kullanilmaz.
    Zaman damgali bekleme icin ``_wait_until`` degiskeni kullanilir.

    Kullanim::

        mission = MissionControl(serial_send_fn, socketio_emit_fn)
        # serial_reader_thread icinde:
        mission.update(telemetry, qr_result, line_result, fused_state)
    """

    # ── Konfigurasyonu ──
    QR_APPROACH_SLOW_DIST_M  = 0.80   # Bu mesafeden itibaren yavasla
    QR_STOP_DIST_M           = 0.25   # Bu mesafede dur
    QR_ALIGN_OFFSET_CM       = 3.0    # Yatay hizalama toleransi (cm)
    STOP_SETTLE_SEC          = 0.8    # Durma sonrasi stabilizasyon (saniye)
    FORKLIFT_TIMEOUT_SEC     = 6.0    # Forklift islem zaman asimi
    RESUME_DELAY_SEC         = 1.0    # Goreve donus oncesi bekleme
    QR_DEBOUNCE_SEC          = 3.0    # Ayni QR'i tekrar isle zamani
    SLOW_SPEED               = 80     # Yavaslatilmis hiz (Teensy motor birimi)

    def __init__(self, serial_send_fn=None, socketio_emit_fn=None):
        """
        Args:
            serial_send_fn: Teensy'ye JSON dict gonderen fonksiyon.
                            Imza: serial_send_fn(dict) -> bool
            socketio_emit_fn: GUI'ye WebSocket event gonderen fonksiyon.
                              Imza: socketio_emit_fn(event_name, data_dict)
        """
        self._send = serial_send_fn or (lambda d: None)
        self._emit = socketio_emit_fn or (lambda e, d: None)

        # Durum
        self._state = MissionState.IDLE
        self._prev_state = MissionState.IDLE
        self._state_enter_time = 0.0

        # Gorev surasi
        self._task_queue = []         # [(qr_data, qr_result), ...]
        self._current_task = None     # Suanki gorev: {"qr_data": ..., "action": "AL"|"BIRAK"}
        self._completed_tasks = []    # Tamamlanmis gorevler

        # QR debounce — ayni QR'i cok kisa surede tekrar islememek icin
        self._processed_qrs = {}      # {qr_data: last_process_time}

        # Bekleme zamanlayicisi
        self._wait_until = 0.0

        # Forklift onay bayragi
        self._forklift_done = False

        # QR yaklasma verisi (son gorulen QR)
        self._approach_qr_data = ""
        self._approach_qr_distance = 999.0
        self._approach_qr_offset = 0.0

        # Istatistikler
        self._mission_start_time = 0.0
        self._total_pickups = 0
        self._total_drops = 0

        # Log
        self._log("[MISSION] MissionControl baslatildi")

    # ------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------

    def start_mission(self):
        """Otonom gorevi baslat (GUI'den cagirilir)."""
        if self._state in (MissionState.IDLE, MissionState.FINISHED, MissionState.ERROR):
            self._mission_start_time = time.time()
            self._completed_tasks.clear()
            self._processed_qrs.clear()
            self._total_pickups = 0
            self._total_drops = 0
            self._log("[MISSION] GOREV BASLADI")
            self._enter_state(MissionState.LINE_FOLLOWING)
            # Teensy'ye cizgi takip baslat komutu
            self._send({"cmd": "start"})

    def abort_mission(self):
        """Gorevi iptal et (GUI'den cagirilir)."""
        self._log("[MISSION] GOREV IPTAL EDILDI")
        self._send({"cmd": "stop"})
        self._enter_state(MissionState.IDLE)

    def notify_forklift_done(self):
        """Teensy'den gelen forklift_done onayi."""
        self._forklift_done = True
        self._log("[MISSION] Forklift islemi tamamlandi (Teensy onay)")

    def update(self, telemetry, qr_result, line_result, fused_state):
        """Her telemetri dongusunde cagrilir (non-blocking).

        Args:
            telemetry: Teensy'den gelen ham telemetri dict
            qr_result: camera.get_qr_result() ciktisi
            line_result: camera.get_line_result() ciktisi
            fused_state: fusion.get_state() ciktisi (RobotState)
        """
        now = time.time()

        if self._state == MissionState.IDLE:
            pass  # Komut bekleniyor

        elif self._state == MissionState.LINE_FOLLOWING:
            self._handle_line_following(qr_result, line_result, now)

        elif self._state == MissionState.QR_DETECTED:
            self._handle_qr_detected(qr_result, now)

        elif self._state == MissionState.APPROACHING_QR:
            self._handle_approaching_qr(qr_result, now)

        elif self._state == MissionState.STOPPING:
            self._handle_stopping(now)

        elif self._state == MissionState.EXECUTING_AL:
            self._handle_executing_al(now)

        elif self._state == MissionState.EXECUTING_BIRAK:
            self._handle_executing_birak(now)

        elif self._state == MissionState.WAITING_ACTION:
            self._handle_waiting_action(now)

        elif self._state == MissionState.RESUMING:
            self._handle_resuming(now)

        elif self._state == MissionState.FINISHED:
            pass  # Gorev bitti

        elif self._state == MissionState.ERROR:
            pass  # Hata durumu — abort veya reset bekleniyor

        # GUI'ye durum yayinla
        self._broadcast_state(telemetry)

    def get_state(self):
        """Suanki gorev durumunu dondur."""
        return self._state

    def get_state_dict(self):
        """Gorev durumunu JSON-serializable dict olarak dondur."""
        elapsed = 0.0
        if self._mission_start_time > 0:
            elapsed = time.time() - self._mission_start_time

        return {
            "state": self._state.value,
            "current_task": self._current_task,
            "completed_count": len(self._completed_tasks),
            "total_pickups": self._total_pickups,
            "total_drops": self._total_drops,
            "elapsed_sec": round(elapsed, 1),
            "approach_qr": self._approach_qr_data,
            "approach_dist": round(self._approach_qr_distance, 3),
            "approach_offset": round(self._approach_qr_offset, 1),
        }

    # ------------------------------------------------------------
    #  State Handlers (Non-Blocking)
    # ------------------------------------------------------------

    def _handle_line_following(self, qr_result, line_result, now):
        """LINE_FOLLOWING: Cizgi takip + QR tarama."""
        # Cizgi takip komutlarini avtech_brain zaten gonderiyor.
        # Burada sadece QR kod olup olmadigini kontrol ediyoruz.

        if not qr_result or not qr_result.get('detected', False):
            return

        qr_data = qr_result.get('data', '').strip()
        if not qr_data:
            return

        # QR debounce kontrolu
        last_processed = self._processed_qrs.get(qr_data, 0)
        if (now - last_processed) < self.QR_DEBOUNCE_SEC:
            return

        # Sadece "AL" veya "BIRAK" komutlarini isle
        qr_upper = qr_data.upper()
        if qr_upper not in ("AL", "BIRAK"):
            self._log(f"[MISSION] QR okundu ama gorev degil: '{qr_data}' (atlaniyor)")
            return

        # QR tespit edildi
        self._approach_qr_data = qr_data
        self._approach_qr_distance = qr_result.get('distance_m', 999.0)
        self._approach_qr_offset = qr_result.get('offset_x_cm', 0.0)

        self._current_task = {
            "qr_data": qr_data,
            "action": qr_upper,
        }

        self._log(f"[MISSION] QR tespit edildi: '{qr_data}' "
                  f"(mesafe: {self._approach_qr_distance:.2f}m, "
                  f"offset: {self._approach_qr_offset:.1f}cm)")

        self._enter_state(MissionState.QR_DETECTED)

    def _handle_qr_detected(self, qr_result, now):
        """QR_DETECTED: QR goruldu, yavaslama baslat."""
        # Teensy'ye yavas ileri komutu gonder
        self._send({"cmd": "slow_forward", "speed": self.SLOW_SPEED})
        self._log("[MISSION] Yavas ileri baslatildi (QR yaklasma)")
        self._enter_state(MissionState.APPROACHING_QR)

    def _handle_approaching_qr(self, qr_result, now):
        """APPROACHING_QR: QR'a hassas yaklasma."""
        # QR hala gorunuyor mu? Guncelle.
        if qr_result and qr_result.get('detected', False):
            qr_data = qr_result.get('data', '').strip()
            # Ayni QR mi kontrolu
            if qr_data.upper() == self._current_task.get("action", ""):
                self._approach_qr_distance = qr_result.get('distance_m', self._approach_qr_distance)
                self._approach_qr_offset = qr_result.get('offset_x_cm', self._approach_qr_offset)

        # Mesafe ve hizalama kontrolu
        dist_ok = self._approach_qr_distance <= self.QR_STOP_DIST_M
        offset_ok = abs(self._approach_qr_offset) <= self.QR_ALIGN_OFFSET_CM

        if dist_ok and offset_ok:
            # Durma mesafesine geldi ve hizali
            self._send({"cmd": "stop"})
            self._log(f"[MISSION] Durma noktasina ulasildi "
                      f"(dist={self._approach_qr_distance:.2f}m, "
                      f"offset={self._approach_qr_offset:.1f}cm)")
            self._enter_state(MissionState.STOPPING)
        elif dist_ok and not offset_ok:
            # Mesafe uygun ama hizali degil — yine de dur (guvenlik)
            self._send({"cmd": "stop"})
            self._log(f"[MISSION] Mesafe OK ama hizalama hassas degil "
                      f"(offset={self._approach_qr_offset:.1f}cm). Yine de duruluyor.")
            self._enter_state(MissionState.STOPPING)

        # Timeout: QR'a yaklasma cok uzun surerse (10 saniye)
        if self._elapsed_in_state(now) > 10.0:
            self._send({"cmd": "stop"})
            self._log("[MISSION] QR yaklasma zaman asimi, duruluyor")
            self._enter_state(MissionState.STOPPING)

    def _handle_stopping(self, now):
        """STOPPING: Arac tamamen durdu, stabilizasyon bekleniyor."""
        if self._elapsed_in_state(now) >= self.STOP_SETTLE_SEC:
            action = self._current_task.get("action", "")
            if action == "AL":
                self._enter_state(MissionState.EXECUTING_AL)
            elif action == "BIRAK":
                self._enter_state(MissionState.EXECUTING_BIRAK)
            else:
                self._log(f"[MISSION] Bilinmeyen gorev: '{action}' — atlaniyor")
                self._enter_state(MissionState.RESUMING)

    def _handle_executing_al(self, now):
        """EXECUTING_AL: Forklift kaldirma komutu gonder."""
        self._forklift_done = False
        self._send({"cmd": "forklift", "action": "up"})
        self._log("[MISSION] >>> FORKLIFT KALDIRILIYOR (AL)")
        self._enter_state(MissionState.WAITING_ACTION)

    def _handle_executing_birak(self, now):
        """EXECUTING_BIRAK: Forklift indirme komutu gonder."""
        self._forklift_done = False
        self._send({"cmd": "forklift", "action": "down"})
        self._log("[MISSION] >>> FORKLIFT INDIRILIYOR (BIRAK)")
        self._enter_state(MissionState.WAITING_ACTION)

    def _handle_waiting_action(self, now):
        """WAITING_ACTION: Teensy'den forklift_done onayi veya timeout bekle."""
        if self._forklift_done:
            # Onay geldi
            action = self._current_task.get("action", "")
            qr_data = self._current_task.get("qr_data", "")

            if action == "AL":
                self._total_pickups += 1
                self._log(f"[MISSION] YUK ALINDI ({self._total_pickups}. pickup)")
            elif action == "BIRAK":
                self._total_drops += 1
                self._log(f"[MISSION] YUK BIRAKILDI ({self._total_drops}. drop)")

            # Bu QR'i islenilmis olarak isaretle
            self._processed_qrs[qr_data] = now
            self._completed_tasks.append(self._current_task)

            # Gorev tamamen bitti mi?
            if action == "BIRAK":
                # BIRAK islemi tamamlandi — bir dongu tamamlandi
                # Birden fazla AL-BIRAK dongusu icin LINE_FOLLOWING'e donus
                # Simdilik tek dongu: FINISHED
                self._log("[MISSION] AL-BIRAK dongusu tamamlandi")
                self._enter_state(MissionState.RESUMING)
            else:
                # AL islemi bitti, cizgi takibe don (BIRAK QR'ini aramak icin)
                self._enter_state(MissionState.RESUMING)

            self._forklift_done = False
            return

        # Timeout kontrolu
        if self._elapsed_in_state(now) >= self.FORKLIFT_TIMEOUT_SEC:
            self._log("[MISSION] Forklift zaman asimi — timeout ile devam ediliyor")
            # Timeout durumunda da islemi tamamlanmis say
            action = self._current_task.get("action", "")
            qr_data = self._current_task.get("qr_data", "")

            if action == "AL":
                self._total_pickups += 1
            elif action == "BIRAK":
                self._total_drops += 1

            self._processed_qrs[qr_data] = now
            self._completed_tasks.append(self._current_task)

            if action == "BIRAK":
                self._enter_state(MissionState.RESUMING)
            else:
                self._enter_state(MissionState.RESUMING)

    def _handle_resuming(self, now):
        """RESUMING: Goreve donus beklemesi."""
        if self._elapsed_in_state(now) >= self.RESUME_DELAY_SEC:
            # AL ve BIRAK ikisi de tamamlandiysa bitir
            if self._total_pickups > 0 and self._total_drops > 0 and \
               self._total_pickups == self._total_drops:
                elapsed = time.time() - self._mission_start_time
                self._log(f"[MISSION] GOREV TAMAMLANDI "
                          f"(sure: {elapsed:.1f}s, "
                          f"pickup: {self._total_pickups}, "
                          f"drop: {self._total_drops})")
                self._send({"cmd": "stop"})
                self._enter_state(MissionState.FINISHED)
            else:
                # Henuz tamamlanmamis gorevler var, cizgi takibe don
                self._log("[MISSION] Cizgi takibe donuluyor (sonraki QR aranacak)")
                self._send({"cmd": "start"})
                self._enter_state(MissionState.LINE_FOLLOWING)

    # ------------------------------------------------------------
    #  Helpers
    # ------------------------------------------------------------

    def _enter_state(self, new_state):
        """Duruma gec ve zamanlayiciyi sifirla."""
        self._prev_state = self._state
        self._state = new_state
        self._state_enter_time = time.time()
        self._log(f"[MISSION] {self._prev_state.value} -> {new_state.value}")

    def _elapsed_in_state(self, now=None):
        """Suanki durumda gecen sure (saniye)."""
        if now is None:
            now = time.time()
        return now - self._state_enter_time

    def _log(self, msg):
        """Log mesaji yazdir."""
        print(msg)

    def _broadcast_state(self, telemetry=None):
        """GUI'ye gorev durumunu WebSocket ile yayinla."""
        try:
            self._emit('mission_state', self.get_state_dict())
        except Exception:
            pass


# --- Standalone Test ---

if __name__ == '__main__':
    """
    Standalone simulasyon — MissionControl state gecislerini test eder.
    Gercek donanim gerektirmez.
    """
    print("=" * 60)
    print("  AVTECH AGV — MissionControl Standalone Test")
    print("=" * 60)

    # Mock serial_send
    def mock_serial(cmd_dict):
        print(f"  [MOCK TX] {json.dumps(cmd_dict)}")
        return True

    # Mock socketio emit
    def mock_emit(event, data):
        pass  # Sessiz

    mc = MissionControl(serial_send_fn=mock_serial, socketio_emit_fn=mock_emit)

    # Bos telemetri ve fusion state
    dummy_telem = {"state": "LINE_FOLLOW", "spdL": 100, "spdR": 100}
    dummy_line = {"detected": True, "color": "black", "offset_cm": 0.0}
    dummy_fused = None  # Kullanilmiyor bu testte

    print("\n--- 1) Gorevi baslat ---")
    mc.start_mission()

    print("\n--- 2) Cizgi takip (QR yok) ---")
    for i in range(5):
        mc.update(dummy_telem, {}, dummy_line, dummy_fused)
        time.sleep(0.01)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 3) 'AL' QR tespit edildi (uzaktan) ---")
    qr_far = {
        'detected': True,
        'data': 'AL',
        'distance_m': 0.60,
        'angle_deg': 2.0,
        'offset_x_cm': 1.5,
    }
    mc.update(dummy_telem, qr_far, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 4) QR_DETECTED -> APPROACHING_QR ---")
    mc.update(dummy_telem, qr_far, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 5) QR'a yakin (0.20m) ---")
    qr_close = {
        'detected': True,
        'data': 'AL',
        'distance_m': 0.20,
        'angle_deg': 0.5,
        'offset_x_cm': 1.0,
    }
    mc.update(dummy_telem, qr_close, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 6) Stabilizasyon bekleme ---")
    time.sleep(1.0)
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 7) EXECUTING_AL -> WAITING_ACTION ---")
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 8) Forklift tamamlandi ---")
    mc.notify_forklift_done()
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 9) Resume bekleme ---")
    time.sleep(1.2)
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 10) 'BIRAK' QR tespit edildi ---")
    time.sleep(3.5)  # QR debounce suresi
    qr_birak = {
        'detected': True,
        'data': 'BIRAK',
        'distance_m': 0.50,
        'angle_deg': -1.0,
        'offset_x_cm': -2.0,
    }
    mc.update(dummy_telem, qr_birak, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- 11) Yaklasma + Durma ---")
    mc.update(dummy_telem, qr_birak, dummy_line, dummy_fused)  # QR_DETECTED -> APPROACHING
    qr_birak_close = dict(qr_birak)
    qr_birak_close['distance_m'] = 0.18
    qr_birak_close['offset_x_cm'] = 0.5
    mc.update(dummy_telem, qr_birak_close, dummy_line, dummy_fused)  # APPROACHING -> STOPPING
    print(f"  State: {mc.get_state().value}")

    print("\n--- 12) Stabilizasyon + BIRAK ---")
    time.sleep(1.0)
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)  # STOPPING -> EXECUTING_BIRAK
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)  # EXECUTING_BIRAK -> WAITING

    print("\n--- 13) Forklift done + Gorev tamamlandi ---")
    mc.notify_forklift_done()
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)
    time.sleep(1.2)
    mc.update(dummy_telem, {}, dummy_line, dummy_fused)
    print(f"  State: {mc.get_state().value}")

    print("\n--- Sonuc ---")
    state_dict = mc.get_state_dict()
    print(f"  Final State: {state_dict['state']}")
    print(f"  Tamamlanan gorevler: {state_dict['completed_count']}")
    print(f"  Toplam pickup: {state_dict['total_pickups']}")
    print(f"  Toplam drop: {state_dict['total_drops']}")
    print(f"  Gecen sure: {state_dict['elapsed_sec']}s")
    print("\n  BASARILI!" if state_dict['state'] == 'FINISHED' else "\n  BASARISIZ!")
