use alloc::{format, string::String};

use embassy_net::Stack;
use picoserve::io::Read;
use picoserve::request::Request;
use picoserve::response::{IntoResponse, ResponseWriter};
use picoserve::{
    AppBuilder, AppRouter, ResponseSent, Router, routing,
    routing::{RequestHandlerService, parse_path_segment},
};

use crate::config::ObstructionSensitivity;
use crate::controller::{
    CommandSubmission, DeskControllerMode, DeskControllerSnapshot, DeskControllerState, DeskFault,
    StopSubmission,
};
use crate::desk::{DeskMotionState, DeskStopReason};
use crate::units::{PositionCounts, RelativeCounts};

async fn index() -> &'static str {
    "Endpoints: GET /status, POST /home, /stop, /up/<steps>, /down/<steps>, /move/<position>\n"
}

fn home_response(state: &DeskControllerState) -> String {
    submission_response("home", state.submit_home(), state.snapshot())
}

fn stop_response(state: &DeskControllerState) -> String {
    match state.submit_stop() {
        StopSubmission::Accepted => response_body("stop", "accepted", state.snapshot()),
        StopSubmission::IgnoredIdle => response_body("stop", "ignored_idle", state.snapshot()),
    }
}

fn up_response(state: &DeskControllerState, steps: i32) -> String {
    let steps = steps.abs();
    submission_response(
        "up",
        state.submit_move_by(RelativeCounts::new(steps)),
        state.snapshot(),
    )
}

fn down_response(state: &DeskControllerState, steps: i32) -> String {
    let steps = steps.abs();
    submission_response(
        "down",
        state.submit_move_by(RelativeCounts::new(-steps)),
        state.snapshot(),
    )
}

fn move_to_response(state: &DeskControllerState, position: i32) -> String {
    submission_response(
        "move",
        state.submit_move_to(PositionCounts::new(position)),
        state.snapshot(),
    )
}

fn status_response(state: &DeskControllerState) -> String {
    let snapshot = state.snapshot();
    let status = state.status();
    format!(
        "mode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\nhomed={}\nneeds_rehome={}\nmotion={}\nlast_stop_reason={}\nobstruction_sensitivity={}\ntarget_active={}\ntarget_position={}\naverage_position={}\nleft_position={}\nright_position={}\nmin_position={}\nmax_position={}\nskew_counts={}\n",
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
        bool_name(status.homed),
        bool_name(status.needs_rehome),
        motion_name(status.motion),
        stop_reason_name(status.last_stop_reason),
        obstruction_sensitivity_name(state.obstruction_sensitivity()),
        bool_name(status.target_active),
        status.target_position,
        status.average_position,
        status.left_position,
        status.right_position,
        status.min_position,
        status.max_position,
        status.skew_counts,
    )
}

fn submission_response(
    command: &'static str,
    submission: CommandSubmission,
    snapshot: DeskControllerSnapshot,
) -> String {
    let result = match submission {
        CommandSubmission::Accepted => "accepted",
        CommandSubmission::RejectedBusy => "rejected_busy",
        CommandSubmission::RejectedUnhomed => "rejected_unhomed",
        CommandSubmission::RejectedFaulted => "rejected_faulted",
    };

    response_body(command, result, snapshot)
}

fn response_body(
    command: &'static str,
    result: &'static str,
    snapshot: DeskControllerSnapshot,
) -> String {
    format!(
        "command={}\nresult={}\nmode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\n",
        command,
        result,
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
    )
}

fn controller_mode_name(mode: DeskControllerMode) -> &'static str {
    match mode {
        DeskControllerMode::Unhomed => "unhomed",
        DeskControllerMode::Ready => "ready",
        DeskControllerMode::Homing => "homing",
        DeskControllerMode::Moving => "moving",
        DeskControllerMode::Faulted => "faulted",
    }
}

fn fault_name(fault: Option<DeskFault>) -> &'static str {
    match fault {
        None => "none",
        Some(DeskFault::LeftLeg(error)) => match error {
            crate::leg::LegError::HomingStartTimeout => "left_leg_homing_start_timeout",
            crate::leg::LegError::PolarityMismatch => "left_leg_polarity_mismatch",
            crate::leg::LegError::MoveTimeout => "left_leg_move_timeout",
        },
        Some(DeskFault::RightLeg(error)) => match error {
            crate::leg::LegError::HomingStartTimeout => "right_leg_homing_start_timeout",
            crate::leg::LegError::PolarityMismatch => "right_leg_polarity_mismatch",
            crate::leg::LegError::MoveTimeout => "right_leg_move_timeout",
        },
        Some(DeskFault::MoveTimeout) => "move_timeout",
        Some(DeskFault::SkewFault) => "skew_fault",
        Some(DeskFault::RehomeRequired) => "rehome_required",
    }
}

fn motion_name(motion: DeskMotionState) -> &'static str {
    match motion {
        DeskMotionState::Idle => "idle",
        DeskMotionState::Homing => "homing",
        DeskMotionState::MovingUp => "moving_up",
        DeskMotionState::MovingDown => "moving_down",
    }
}

fn stop_reason_name(reason: DeskStopReason) -> &'static str {
    match reason {
        DeskStopReason::None => "none",
        DeskStopReason::TargetReached => "target_reached",
        DeskStopReason::UserStop => "user_stop",
        DeskStopReason::Obstruction => "obstruction",
    }
}

fn obstruction_sensitivity_name(sensitivity: ObstructionSensitivity) -> &'static str {
    match sensitivity {
        ObstructionSensitivity::None => "none",
        ObstructionSensitivity::Low => "low",
        ObstructionSensitivity::Medium => "medium",
        ObstructionSensitivity::High => "high",
    }
}

fn bool_name(value: bool) -> &'static str {
    if value { "true" } else { "false" }
}

#[derive(Clone, Copy)]
struct HomeService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct StopService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct UpService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct DownService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct MoveToService {
    state: &'static DeskControllerState,
}

#[derive(Clone, Copy)]
struct StatusService {
    state: &'static DeskControllerState,
}

impl RequestHandlerService<(), ()> for HomeService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (): (),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        home_response(self.state)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), ()> for StopService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (): (),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        stop_response(self.state)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), (i32,)> for UpService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (steps,): (i32,),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        up_response(self.state, steps)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), (i32,)> for DownService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (steps,): (i32,),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        down_response(self.state, steps)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), (i32,)> for MoveToService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (position,): (i32,),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        move_to_response(self.state, position)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

impl RequestHandlerService<(), ()> for StatusService {
    async fn call_request_handler_service<R: Read, W: ResponseWriter<Error = R::Error>>(
        &self,
        _state: &(),
        (): (),
        request: Request<'_, R>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        status_response(self.state)
            .write_to(request.body_connection.finalize().await?, response_writer)
            .await
    }
}

pub struct Application {
    state: &'static DeskControllerState,
}

impl AppBuilder for Application {
    type PathRouter = impl routing::PathRouter;

    fn build_app(self) -> Router<Self::PathRouter> {
        Router::new()
            .route("/", routing::get(index))
            .route(
                "/status",
                routing::get_service(StatusService { state: self.state }),
            )
            .route(
                "/home",
                routing::post_service(HomeService { state: self.state }),
            )
            .route(
                "/stop",
                routing::post_service(StopService { state: self.state }),
            )
            .route(
                ("/up", parse_path_segment::<i32>()),
                routing::post_service(UpService { state: self.state }),
            )
            .route(
                ("/down", parse_path_segment::<i32>()),
                routing::post_service(DownService { state: self.state }),
            )
            .route(
                ("/move", parse_path_segment::<i32>()),
                routing::post_service(MoveToService { state: self.state }),
            )
    }
}

pub const WEB_TASK_POOL_SIZE: usize = 2;

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
pub async fn web_task(
    id: usize,
    stack: Stack<'static>,
    router: &'static AppRouter<Application>,
    config: &'static picoserve::Config,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::Server::new(router, config, &mut http_buffer)
        .listen_and_serve(id, stack, port, &mut tcp_rx_buffer, &mut tcp_tx_buffer)
        .await
        .into_never()
}

pub struct WebApp {
    pub router: &'static Router<<Application as AppBuilder>::PathRouter>,
    pub config: &'static picoserve::Config,
}

impl WebApp {
    pub fn new(state: &'static DeskControllerState) -> Self {
        let router =
            picoserve::make_static!(AppRouter<Application>, Application { state }.build_app());
        let config = picoserve::make_static!(
            picoserve::Config,
            picoserve::Config::new(Default::default()).keep_connection_alive()
        );

        Self { router, config }
    }
}
