"""Conduz o treino automático pelo mesmo fluxo de propostas da interface."""


class AutoOptimizer:
    def __init__(self, app):
        self.app = app
        self.active = False
        self.stopping = False
        self.job = self.future = None
        self.awaiting_decision = False
        self.tested = self.accepted = 0

    def start(self):
        if self.active or self.app.profile is None:
            return
        self.active = True
        self.tested = self.accepted = 0
        self.show("Iniciando")
        self.app.update_buttons()
        self.schedule()

    def pause(self):
        if self.active:
            self.stopping = True
            self.show("Pausa solicitada")
            self.app.update_buttons()

    def cancel(self):
        if self.job is not None:
            self.app.root.after_cancel(self.job)
        self.job = self.future = None
        self.awaiting_decision = self.stopping = self.active = False

    def show(self, message):
        self.app.status.set(f"Otimização · {message} · propostas testadas {self.tested} · aceitas {self.accepted}")

    def finish(self, message):
        self.cancel()
        self.show(message)
        self.app.update_buttons()

    def schedule(self, delay=10):
        self.job = self.app.root.after(delay, self.advance)

    def advance(self):
        self.job = None
        try:
            self.step()
        except Exception as error:
            self.finish(f"Interrompida · {error}")

    def step(self):
        app = self.app
        if self.future is not None:
            if not self.future.done():
                self.schedule(80)
                return
            result = self.future.result()
            self.future = None
            app.accept_auto_validation(result)
            self.awaiting_decision = True
        if self.awaiting_decision:
            if app.confirmation_stop is not None:
                self.show("Concluindo a confirmação antes da pausa" if self.stopping else "Confirmando em 3 sementes")
                self.schedule(100)
                return
            self.awaiting_decision = False
            decision = app.iterations[app.last_iteration_id].decision
            if decision == "Confirmação interrompida":
                self.finish(app.run_summary)
                return
            if decision in ("Aceita", "Rejeitada"):
                self.tested += 1
                self.accepted += int(decision == "Aceita")
            self.show(decision)
            self.schedule()
            return
        if self.stopping:
            self.finish("Pausada")
            return
        # Uma tomada oficial tem sua própria distância, ruído e cronometragem.
        if app.pending_id is None and app.last_iteration_id is not None and app.validation is not None:
            if app.proposal is None:
                app.run_diagnosis()
            if app.proposal is None:
                self.finish("Concluída · diagnóstico sem nova proposta")
                return
            if any(it.reference_id == app.reference_id and it.decision == "Rejeitada"
                   and it.profile == app.proposal for it in app.iterations[app.map_history_start:]):
                self.finish("Concluída · proposta já testada com esta referência")
                return
            app.apply_proposal()
        index = app.pending_id if app.pending_id is not None else len(app.iterations)
        self.show(f"Simulando a iteração #{index+1:02d}")
        self.future = app.start_auto_validation()
        self.schedule(80)
